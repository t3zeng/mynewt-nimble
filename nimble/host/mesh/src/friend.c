/*  Bluetooth Mesh */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "syscfg/syscfg.h"
#if MYNEWT_VAL(BLE_MESH_FRIEND) == 1

#include <stdint.h>
#include <errno.h>
#include <assert.h>

#define BT_DBG_ENABLED (MYNEWT_VAL(BLE_MESH_DEBUG_FRIEND))
#include "host/ble_hs_log.h"

#include "mesh/mesh.h"
#include "mesh/slist.h"
#include "mesh_priv.h"
#include "crypto.h"
#include "adv.h"
#include "net.h"
#include "transport.h"
#include "access.h"
#include "foundation.h"
#include "friend.h"

#define FRIEND_BUF_SIZE     (BT_MESH_ADV_DATA_SIZE - BT_MESH_NET_HDR_LEN)

/* We reserve one extra buffer for each friendship, since we need to be able
 * to resend the last sent PDU, which sits separately outside of the queue.
 */
#define FRIEND_BUF_COUNT ((MYNEWT_VAL(BLE_MESH_FRIEND_QUEUE_SIZE) + 1) * MYNEWT_VAL(BLE_MESH_FRIEND_LPN_COUNT))

static os_membuf_t friend_buf_mem[OS_MEMPOOL_SIZE(
		MYNEWT_VAL(BLE_MESH_ADV_BUF_COUNT),
		BT_MESH_ADV_DATA_SIZE + BT_MESH_ADV_USER_DATA_SIZE)];

struct os_mbuf_pool friend_os_mbuf_pool;
static struct os_mempool friend_buf_mempool;


struct friend_pdu_info {
	u16_t  src;
	u16_t  dst;

	u8_t   seq[3];

	u8_t   ttl:7,
	       ctl:1;

	u32_t  iv_index;
};

static u64_t buf_seq_auth[FRIEND_BUF_COUNT];

/* Helpers to access the storage array, since we don't have access to its
 * type at this point anymore.
 */

#define BUF_SIZE(pool) (pool->omp_pool->mp_block_size)


int net_buf_id(struct os_mbuf *buf)
{
	struct os_mbuf_pool *pool = buf->om_omp;
	u8_t *pool_start = (u8_t *)pool->omp_pool->mp_membuf_addr;
	u8_t *buf_ptr = (u8_t *)buf;

	return (buf_ptr - pool_start) / BUF_SIZE(pool);
}

static void discard_buffer(void)
{
	struct bt_mesh_friend *frnd = &bt_mesh.frnd[0];
	struct os_mbuf *buf;
	int i;

	/* Find the Friend context with the most queued buffers */
	for (i = 1; i < ARRAY_SIZE(bt_mesh.frnd); i++) {
		if (bt_mesh.frnd[i].queue_size > frnd->queue_size) {
			frnd = &bt_mesh.frnd[i];
		}
	}

	BT_WARN("Discarding buffer for LPN 0x%04x", frnd->lpn);

	buf = net_buf_slist_get(&frnd->queue);
	__ASSERT_NO_MSG(buf != NULL);
	net_buf_unref(buf);
}

static struct os_mbuf *friend_buf_alloc(u16_t src)
{
	u8_t xmit = bt_mesh_net_transmit_get();
	struct os_mbuf *buf;

	do {
		buf = bt_mesh_adv_create_from_pool(&friend_os_mbuf_pool,
						   BT_MESH_ADV_DATA,
						   BT_MESH_TRANSMIT_COUNT(xmit),
						   BT_MESH_TRANSMIT_INT(xmit),
						   K_NO_WAIT);
		if (!buf) {
			discard_buffer();
		}
	} while (!buf);

	BT_MESH_ADV(buf)->addr = src;
	buf_seq_auth[net_buf_id(buf)] = TRANS_SEQ_AUTH_NVAL;

	BT_DBG("allocated buf %p", buf);

	return buf;
}

struct bt_mesh_friend *bt_mesh_friend_find(u16_t net_idx, u16_t lpn_addr,
					   bool valid, bool established)
{
	int i;

	BT_DBG("net_idx 0x%04x lpn_addr 0x%04x", net_idx, lpn_addr);

	for (i = 0; i < ARRAY_SIZE(bt_mesh.frnd); i++) {
		struct bt_mesh_friend *frnd = &bt_mesh.frnd[i];

		if (valid && !frnd->valid) {
			continue;
		}

		if (established && !frnd->established) {
			continue;
		}

		if (net_idx != BT_MESH_KEY_ANY && frnd->net_idx != net_idx) {
			continue;
		}

		if (frnd->lpn == lpn_addr) {
			return frnd;
		}
	}

	return NULL;
}

/* Intentionally start a little bit late into the ReceiveWindow when
 * it's large enough. This may improve reliability with some platforms,
 * like the PTS, where the receiver might not have sufficiently compensated
 * for internal latencies required to start scanning.
 */
static s32_t recv_delay(struct bt_mesh_friend *frnd)
{
#if CONFIG_BT_MESH_FRIEND_RECV_WIN > 50
	return (s32_t)frnd->recv_delay + (CONFIG_BT_MESH_FRIEND_RECV_WIN / 5);
#else
	return frnd->recv_delay;
#endif
}

static void friend_clear(struct bt_mesh_friend *frnd)
{
	int i;

	BT_DBG("LPN 0x%04x", frnd->lpn);

	k_delayed_work_cancel(&frnd->timer);

	bt_mesh_friend_cred_del(bt_mesh.sub[0].net_idx, frnd->lpn);

	if (frnd->last) {
		net_buf_unref(frnd->last);
		frnd->last = NULL;
	}

	while (!sys_slist_is_empty(&frnd->queue)) {
		net_buf_unref(net_buf_slist_get(&frnd->queue));
	}

	for (i = 0; i < ARRAY_SIZE(frnd->seg); i++) {
		struct bt_mesh_friend_seg *seg = &frnd->seg[i];

		while (!sys_slist_is_empty(&seg->queue)) {
			net_buf_unref(net_buf_slist_get(&seg->queue));
		}
	}

	frnd->valid = 0;
	frnd->established = 0;
	frnd->pending_buf = 0;
	frnd->fsn = 0;
	frnd->queue_size = 0;
	memset(frnd->sub_list, 0, sizeof(frnd->sub_list));
}

void bt_mesh_friend_clear_net_idx(u16_t net_idx)
{
	int i;

	BT_DBG("net_idx 0x%04x", net_idx);

	for (i = 0; i < ARRAY_SIZE(bt_mesh.frnd); i++) {
		struct bt_mesh_friend *frnd = &bt_mesh.frnd[i];

		if (frnd->net_idx == BT_MESH_KEY_UNUSED) {
			continue;
		}

		if (net_idx == BT_MESH_KEY_ANY || frnd->net_idx == net_idx) {
			friend_clear(frnd);
		}
	}
}

void bt_mesh_friend_sec_update(u16_t net_idx)
{
	int i;

	BT_DBG("net_idx 0x%04x", net_idx);

	for (i = 0; i < ARRAY_SIZE(bt_mesh.frnd); i++) {
		struct bt_mesh_friend *frnd = &bt_mesh.frnd[i];

		if (frnd->net_idx == BT_MESH_KEY_UNUSED) {
			continue;
		}

		if (net_idx == BT_MESH_KEY_ANY || frnd->net_idx == net_idx) {
			frnd->sec_update = 1;
		}
	}
}

int bt_mesh_friend_clear(struct bt_mesh_net_rx *rx, struct os_mbuf *buf)
{
	struct bt_mesh_ctl_friend_clear *msg = (void *)buf->om_data;
	struct bt_mesh_friend *frnd;
	u16_t lpn_addr, lpn_counter;
	struct bt_mesh_net_tx tx = {
		.sub  = rx->sub,
		.ctx  = &rx->ctx,
		.src  = bt_mesh_primary_addr(),
		.xmit = bt_mesh_net_transmit_get(),
	};
	struct bt_mesh_ctl_friend_clear_confirm cfm;

	if (buf->om_len < sizeof(*msg)) {
		BT_WARN("Too short Friend Clear");
		return -EINVAL;
	}

	lpn_addr = sys_be16_to_cpu(msg->lpn_addr);
	lpn_counter = sys_be16_to_cpu(msg->lpn_counter);

	BT_DBG("LPN addr 0x%04x counter 0x%04x", lpn_addr, lpn_counter);

	frnd = bt_mesh_friend_find(rx->sub->net_idx, lpn_addr, false, false);
	if (!frnd) {
		BT_WARN("No matching LPN addr 0x%04x", lpn_addr);
		return 0;
	}

	/* A Friend Clear message is considered valid if the result of the
	 * subtraction of the value of the LPNCounter field of the Friend
	 * Request message (the one that initiated the friendship) from the
	 * value of the LPNCounter field of the Friend Clear message, modulo
	 * 65536, is in the range 0 to 255 inclusive.
	 */
	if (lpn_counter - frnd->lpn_counter > 255) {
		BT_WARN("LPN Counter out of range (old %u new %u)",
			frnd->lpn_counter, lpn_counter);
		return 0;
	}

	tx.ctx->send_ttl = BT_MESH_TTL_MAX;

	cfm.lpn_addr    = msg->lpn_addr;
	cfm.lpn_counter = msg->lpn_counter;

	bt_mesh_ctl_send(&tx, TRANS_CTL_OP_FRIEND_CLEAR_CFM, &cfm,
			 sizeof(cfm), NULL, NULL);

	friend_clear(frnd);

	return 0;
}

static void friend_sub_add(struct bt_mesh_friend *frnd, u16_t addr)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(frnd->sub_list); i++) {
		if (frnd->sub_list[i] == BT_MESH_ADDR_UNASSIGNED) {
			frnd->sub_list[i] = addr;
			return;
		}
	}

	BT_WARN("No space in friend subscription list");
}

static void friend_sub_rem(struct bt_mesh_friend *frnd, u16_t addr)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(frnd->sub_list); i++) {
		if (frnd->sub_list[i] == addr) {
			frnd->sub_list[i] = BT_MESH_ADDR_UNASSIGNED;
			return;
		}
	}
}

static struct os_mbuf *create_friend_pdu(struct bt_mesh_friend *frnd,
					 struct friend_pdu_info *info,
					 struct os_mbuf *sdu)
{
	struct bt_mesh_subnet *sub;
	const u8_t *enc, *priv;
	struct os_mbuf *buf;
	u8_t nid, key_idx;

	sub = bt_mesh_subnet_get(frnd->net_idx);
	__ASSERT_NO_MSG(sub != NULL);

	buf = friend_buf_alloc(info->src);

	key_idx = (sub->kr_phase == BT_MESH_KR_PHASE_2);

	/* Friend Offer needs master security credentials */
	if (info->ctl && TRANS_CTL_OP(sdu->om_data) == TRANS_CTL_OP_FRIEND_OFFER) {
		enc = sub->keys[key_idx].enc;
		priv = sub->keys[key_idx].privacy;
		nid = sub->keys[key_idx].nid;
	} else {
		if (bt_mesh_friend_cred_get(sub->net_idx, frnd->lpn, key_idx,
					    &nid, &enc, &priv)) {
			BT_ERR("bt_mesh_friend_cred_get failed");
			goto failed;
		}
	}

	net_buf_add_u8(buf, (nid | (info->iv_index & 1) << 7));

	if (info->ctl) {
		net_buf_add_u8(buf, info->ttl | 0x80);
	} else {
		net_buf_add_u8(buf, info->ttl);
	}

	net_buf_add_mem(buf, info->seq, sizeof(info->seq));

	net_buf_add_be16(buf, info->src);
	net_buf_add_be16(buf, info->dst);

	net_buf_add_mem(buf, sdu->om_data, sdu->om_len);

	/* We re-encrypt and obfuscate using the received IVI rather than
	 * the normal TX IVI (which may be different) since the transport
	 * layer nonce includes the IVI.
	 */
	if (bt_mesh_net_encrypt(enc, buf, info->iv_index, false)) {
		BT_ERR("Re-encrypting failed");
		goto failed;
	}

	if (bt_mesh_net_obfuscate(buf->om_data, info->iv_index, priv)) {
		BT_ERR("Re-obfuscating failed");
		goto failed;
	}

	return buf;

failed:
	net_buf_unref(buf);
	return NULL;
}

static struct os_mbuf *encode_friend_ctl(struct bt_mesh_friend *frnd,
					 u8_t ctl_op,
					 struct os_mbuf *sdu)
{
	struct friend_pdu_info info;

	BT_DBG("LPN 0x%04x", frnd->lpn);

	net_buf_simple_push_u8(sdu, TRANS_CTL_HDR(ctl_op, 0));

	info.src = bt_mesh_primary_addr();
	info.dst = frnd->lpn;

	info.ctl = 1;
	info.ttl = 0;

	info.seq[0] = (bt_mesh.seq >> 16);
	info.seq[1] = (bt_mesh.seq >> 8);
	info.seq[2] = bt_mesh.seq++;

	info.iv_index = BT_MESH_NET_IVI_TX;

	return create_friend_pdu(frnd, &info, sdu);
}

static struct os_mbuf *encode_update(struct bt_mesh_friend *frnd, u8_t md)
{
	struct bt_mesh_ctl_friend_update *upd;
	struct os_mbuf *sdu = NET_BUF_SIMPLE(1 + sizeof(*upd));
	struct bt_mesh_subnet *sub = bt_mesh_subnet_get(frnd->net_idx);

	__ASSERT_NO_MSG(sub != NULL);

	BT_DBG("lpn 0x%04x md 0x%02x", frnd->lpn, md);

	net_buf_simple_init(sdu, 1);

	upd = net_buf_simple_add(sdu, sizeof(*upd));
	upd->flags = bt_mesh_net_flags(sub);
	upd->iv_index = sys_cpu_to_be32(bt_mesh.iv_index);
	upd->md = md;

	return encode_friend_ctl(frnd, TRANS_CTL_OP_FRIEND_UPDATE, sdu);
}

static void enqueue_sub_cfm(struct bt_mesh_friend *frnd, u8_t xact)
{
	struct bt_mesh_ctl_friend_sub_confirm *cfm;
	struct os_mbuf *sdu = NET_BUF_SIMPLE(1 + sizeof(*cfm));
	struct os_mbuf *buf;

	BT_DBG("lpn 0x%04x xact 0x%02x", frnd->lpn, xact);

	net_buf_simple_init(sdu, 1);

	cfm = net_buf_simple_add(sdu, sizeof(*cfm));
	cfm->xact = xact;

	buf = encode_friend_ctl(frnd, TRANS_CTL_OP_FRIEND_SUB_CFM, sdu);
	if (!buf) {
		BT_ERR("Unable to encode Subscription List Confirmation");
		return;
	}

	if (frnd->last) {
		BT_DBG("Discarding last PDU");
		net_buf_unref(frnd->last);
	}

	frnd->last = buf;
	frnd->send_last = 1;
}

int bt_mesh_friend_sub_add(struct bt_mesh_net_rx *rx,
			   struct os_mbuf *buf)
{
	struct bt_mesh_friend *frnd;
	u8_t xact;

	if (buf->om_len < BT_MESH_FRIEND_SUB_MIN_LEN) {
		BT_WARN("Too short Friend Subscription Add");
		return -EINVAL;
	}

	frnd = bt_mesh_friend_find(rx->sub->net_idx, rx->ctx.addr, true, true);
	if (!frnd) {
		BT_WARN("No matching LPN addr 0x%04x", rx->ctx.addr);
		return 0;
	}

	if (frnd->pending_buf) {
		BT_WARN("Previous buffer not yet sent!");
		return 0;
	}

	k_delayed_work_submit(&frnd->timer, recv_delay(frnd));

	xact = net_buf_simple_pull_u8(buf);

	while (buf->om_len >= 2) {
		friend_sub_add(frnd, net_buf_simple_pull_be16(buf));
	}

	enqueue_sub_cfm(frnd, xact);

	BT_DBG("Waiting RecvDelay of %d ms", recv_delay(frnd));

	return 0;
}

int bt_mesh_friend_sub_rem(struct bt_mesh_net_rx *rx,
			   struct os_mbuf *buf)
{
	struct bt_mesh_friend *frnd;
	u8_t xact;

	if (buf->om_len < BT_MESH_FRIEND_SUB_MIN_LEN) {
		BT_WARN("Too short Friend Subscription Remove");
		return -EINVAL;
	}

	frnd = bt_mesh_friend_find(rx->sub->net_idx, rx->ctx.addr, true, true);
	if (!frnd) {
		BT_WARN("No matching LPN addr 0x%04x", rx->ctx.addr);
		return 0;
	}

	if (frnd->pending_buf) {
		BT_WARN("Previous buffer not yet sent!");
		return 0;
	}

	k_delayed_work_submit(&frnd->timer, recv_delay(frnd));

	xact = net_buf_simple_pull_u8(buf);

	while (buf->om_len >= 2) {
		friend_sub_rem(frnd, net_buf_simple_pull_be16(buf));
	}

	enqueue_sub_cfm(frnd, xact);

	BT_DBG("Waiting RecvDelay of %d ms", recv_delay(frnd));

	return 0;
}

static void enqueue_buf(struct bt_mesh_friend *frnd, struct os_mbuf *buf)
{
	net_buf_slist_put(&frnd->queue, buf);
	frnd->queue_size++;
}

static void enqueue_update(struct bt_mesh_friend *frnd, u8_t md)
{
	struct os_mbuf *buf;

	buf = encode_update(frnd, md);
	if (!buf) {
		BT_ERR("Unable to encode Friend Update");
		return;
	}

	frnd->sec_update = 0;
	enqueue_buf(frnd, buf);
}

int bt_mesh_friend_poll(struct bt_mesh_net_rx *rx, struct os_mbuf *buf)
{
	struct bt_mesh_ctl_friend_poll *msg = (void *)buf->om_data;
	struct bt_mesh_friend *frnd;

	if (buf->om_len < sizeof(*msg)) {
		BT_WARN("Too short Friend Poll");
		return -EINVAL;
	}

	frnd = bt_mesh_friend_find(rx->sub->net_idx, rx->ctx.addr, true, false);
	if (!frnd) {
		BT_WARN("No matching LPN addr 0x%04x", rx->ctx.addr);
		return 0;
	}

	if (msg->fsn & ~1) {
		BT_WARN("Prohibited (non-zero) padding bits");
		return -EINVAL;
	}

	if (frnd->pending_buf) {
		BT_WARN("Previous buffer not yet sent");
		return 0;
	}

	k_delayed_work_submit(&frnd->timer, recv_delay(frnd));

	if (!frnd->established) {
		BT_DBG("Friendship established with 0x%04x", frnd->lpn);
		frnd->established = 1;
	}

	BT_DBG("msg->fsn %u frnd->fsn %u", (msg->fsn & 1), frnd->fsn);

	if (msg->fsn == frnd->fsn && frnd->last) {
		BT_DBG("Re-sending last PDU");
		frnd->send_last = 1;
	} else {
		frnd->fsn = msg->fsn;

		if (sys_slist_is_empty(&frnd->queue)) {
			enqueue_update(frnd, 0);
			BT_DBG("Enqueued Friend Update to empty queue");
		}
	}

	BT_DBG("Waiting RecvDelay of %d ms", recv_delay(frnd));

	return 0;
}

static struct bt_mesh_friend *find_clear(u16_t prev_friend)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bt_mesh.frnd); i++) {
		struct bt_mesh_friend *frnd = &bt_mesh.frnd[i];

		if (frnd->clear.frnd == prev_friend) {
			return frnd;
		}
	}

	return NULL;
}

static void friend_clear_sent(struct os_mbuf *buf, u16_t duration, int err)
{
	struct bt_mesh_friend *frnd;

	BT_DBG("addr 0x%02x", BT_MESH_ADV(buf)->addr);

	frnd = find_clear(BT_MESH_ADV(buf)->addr);
	if (!frnd) {
		BT_WARN("No matching clear procedure found for 0x%02x",
			BT_MESH_ADV(buf)->addr);
		return;
	}

	k_delayed_work_submit(&frnd->clear.timer,
			      duration + K_SECONDS(frnd->clear.repeat_sec));
	frnd->clear.repeat_sec *= 2;
}

static void send_friend_clear(struct bt_mesh_friend *frnd)
{
	struct bt_mesh_msg_ctx ctx = {
		.net_idx  = frnd->net_idx,
		.app_idx  = BT_MESH_KEY_UNUSED,
		.addr     = frnd->clear.frnd,
		.send_ttl = BT_MESH_TTL_MAX,
	};
	struct bt_mesh_net_tx tx = {
		.sub  = &bt_mesh.sub[0],
		.ctx  = &ctx,
		.src  = bt_mesh_primary_addr(),
		.xmit = bt_mesh_net_transmit_get(),
	};
	struct bt_mesh_ctl_friend_clear req = {
		.lpn_addr    = sys_cpu_to_be16(frnd->lpn),
		.lpn_counter = sys_cpu_to_be16(frnd->lpn_counter),
	};

	BT_DBG("");

	bt_mesh_ctl_send(&tx, TRANS_CTL_OP_FRIEND_CLEAR, &req,
			 sizeof(req), NULL, friend_clear_sent);
}

static void clear_timeout(struct os_event *work)
{
	struct bt_mesh_friend *frnd = work->ev_arg;
	u32_t now, duration;

	BT_DBG("LPN 0x%04x (old) Friend 0x%04x", frnd->lpn, frnd->clear.frnd);

	now = k_uptime_get_32();
	/* Handle time wrap-around due to 32-bit limit */
	if (now < frnd->clear.start) {
		duration = (UINT32_MAX - frnd->clear.start) + now;
	} else {
		duration = now - frnd->clear.start;
	}

	if (duration > 2 * frnd->poll_to) {
		BT_DBG("Clear Procedure timer expired");
		frnd->clear.frnd = BT_MESH_ADDR_UNASSIGNED;
		return;
	}

	send_friend_clear(frnd);
}

static void clear_procedure_start(struct bt_mesh_friend *frnd)
{
	BT_DBG("LPN 0x%04x (old) Friend 0x%04x", frnd->lpn, frnd->clear.frnd);

	frnd->clear.start = k_uptime_get_32() + (2 * frnd->poll_to);
	frnd->clear.repeat_sec = 1;

	send_friend_clear(frnd);
}

int bt_mesh_friend_clear_cfm(struct bt_mesh_net_rx *rx,
			     struct os_mbuf *buf)
{
	struct bt_mesh_ctl_friend_clear_confirm *msg = (void *)buf->om_data;
	struct bt_mesh_friend *frnd;
	u16_t lpn_addr, lpn_counter;

	BT_DBG("");

	if (buf->om_len < sizeof(*msg)) {
		BT_WARN("Too short Friend Clear Confirm");
		return -EINVAL;
	}

	frnd = find_clear(rx->ctx.addr);
	if (!frnd) {
		BT_WARN("No pending clear procedure for 0x%02x", rx->ctx.addr);
		return 0;
	}

	lpn_addr = sys_be16_to_cpu(msg->lpn_addr);
	if (lpn_addr != frnd->lpn) {
		BT_WARN("LPN address mismatch (0x%04x != 0x%04x)",
			lpn_addr, frnd->lpn);
		return 0;
	}

	lpn_counter = sys_be16_to_cpu(msg->lpn_counter);
	if (lpn_counter != frnd->lpn_counter) {
		BT_WARN("LPN counter mismatch (0x%04x != 0x%04x)",
			lpn_counter, frnd->lpn_counter);
		return 0;
	}

	k_delayed_work_cancel(&frnd->clear.timer);
	frnd->clear.frnd = BT_MESH_ADDR_UNASSIGNED;

	return 0;
}

static void enqueue_offer(struct bt_mesh_friend *frnd, s8_t rssi)
{
	struct bt_mesh_ctl_friend_offer *off;
	struct os_mbuf *sdu = NET_BUF_SIMPLE(1 + sizeof(*off));
	struct os_mbuf *buf;

	BT_DBG("");

	net_buf_simple_init(sdu, 1);

	off = net_buf_simple_add(sdu, sizeof(*off));

	off->recv_win = CONFIG_BT_MESH_FRIEND_RECV_WIN,
	off->queue_size = CONFIG_BT_MESH_FRIEND_QUEUE_SIZE,
	off->sub_list_size = ARRAY_SIZE(frnd->sub_list),
	off->rssi = rssi,
	off->frnd_counter = sys_cpu_to_be16(frnd->counter);

	buf = encode_friend_ctl(frnd, TRANS_CTL_OP_FRIEND_OFFER, sdu);
	if (!buf) {
		BT_ERR("Unable to encode Friend Offer");
		return;
	}

	frnd->counter++;

	enqueue_buf(frnd, buf);
}

#define RECV_WIN                  CONFIG_BT_MESH_FRIEND_RECV_WIN
#define RSSI_FACT(crit)           (((crit) >> 5) & (u8_t)BIT_MASK(2))
#define RECV_WIN_FACT(crit)       (((crit) >> 3) & (u8_t)BIT_MASK(2))
#define MIN_QUEUE_SIZE_LOG(crit)  ((crit) & (u8_t)BIT_MASK(3))
#define MIN_QUEUE_SIZE(crit)      ((u32_t)BIT(MIN_QUEUE_SIZE_LOG(crit)))

static s32_t offer_delay(struct bt_mesh_friend *frnd, s8_t rssi, u8_t crit)
{
	/* Scaling factors. The actual values are 1, 1.5, 2 & 2.5, but we
	 * want to avoid floating-point arithmetic.
	 */
	static const u8_t fact[] = { 10, 15, 20, 25 };
	s32_t delay;

	BT_DBG("ReceiveWindowFactor %u ReceiveWindow %u RSSIFactor %u RSSI %d",
	       fact[RECV_WIN_FACT(crit)], RECV_WIN,
	       fact[RSSI_FACT(crit)], rssi);

	/* Delay = ReceiveWindowFactor * ReceiveWindow - RSSIFactor * RSSI */
	delay = (s32_t)fact[RECV_WIN_FACT(crit)] * RECV_WIN;
	delay -= (s32_t)fact[RSSI_FACT(crit)] * rssi;
	delay /= 10;

	BT_DBG("Local Delay calculated as %d ms", delay);

	if (delay < 100) {
		return K_MSEC(100);
	}

	return K_MSEC(delay);
}

int bt_mesh_friend_req(struct bt_mesh_net_rx *rx, struct os_mbuf *buf)
{
	struct bt_mesh_ctl_friend_req *msg = (void *)buf->om_data;
	struct bt_mesh_subnet *sub = rx->sub;
	struct bt_mesh_friend *frnd = NULL;
	u16_t old_friend;
	u16_t poll_to;
	int i;

	if (buf->om_len < sizeof(*msg)) {
		BT_WARN("Too short Friend Request");
		return -EINVAL;
	}

	if (msg->recv_delay <= 0x09) {
		BT_WARN("Prohibited ReceiveDelay (0x%02x)", msg->recv_delay);
		return -EINVAL;
	}

	poll_to = (((u32_t)msg->poll_to[0] << 16) |
		   ((u32_t)msg->poll_to[1] << 8) |
		   ((u32_t)msg->poll_to[2]));

	if (poll_to <= 0x000009 || poll_to >= 0x34bc00) {
		BT_WARN("Prohibited PollTimeout (0x%06x)", poll_to);
		return -EINVAL;
	}

	if (msg->num_elem == 0x00) {
		BT_WARN("Prohibited NumElements value (0x00)");
		return -EINVAL;
	}

	if (!MIN_QUEUE_SIZE_LOG(msg->criteria)) {
		BT_WARN("Prohibited Minimum Queue Size in Friend Request");
		return -EINVAL;
	}

	if (CONFIG_BT_MESH_FRIEND_QUEUE_SIZE < MIN_QUEUE_SIZE(msg->criteria)) {
		BT_WARN("We have a too small Friend Queue size (%u < %u)",
			CONFIG_BT_MESH_FRIEND_QUEUE_SIZE,
			MIN_QUEUE_SIZE(msg->criteria));
		return 0;
	}

	old_friend = sys_be16_to_cpu(msg->prev_addr);
	if (BT_MESH_ADDR_IS_UNICAST(old_friend)) {
		frnd = bt_mesh_friend_find(rx->sub->net_idx, old_friend,
					   true, true);
	} else {
		frnd = bt_mesh_friend_find(rx->sub->net_idx, rx->ctx.addr,
					   true, true);
	}

	if (frnd) {
		BT_WARN("Existing LPN re-requesting Friendship");
		friend_clear(frnd);
		goto init_friend;
	}

	for (i = 0; i < ARRAY_SIZE(bt_mesh.frnd); i++) {
		if (!bt_mesh.frnd[i].valid) {
			frnd = &bt_mesh.frnd[i];
			frnd->valid = 1;
			break;
		}
	}

	if (!frnd) {
		BT_WARN("No free Friend contexts for new LPN");
		return -ENOMEM;
	}

init_friend:
	frnd->lpn = rx->ctx.addr;
	frnd->net_idx = rx->sub->net_idx;
	frnd->recv_delay = msg->recv_delay;
	frnd->poll_to = poll_to * 100;
	frnd->lpn_counter = sys_be16_to_cpu(msg->lpn_counter);
	frnd->clear.frnd = sys_be16_to_cpu(msg->prev_addr);

	BT_DBG("LPN 0x%04x rssi %d recv_delay %u poll_to %ums",
	       frnd->lpn, rx->rssi, frnd->recv_delay, frnd->poll_to);

	if (BT_MESH_ADDR_IS_UNICAST(old_friend) &&
	    !bt_mesh_elem_find(old_friend)) {
		clear_procedure_start(frnd);
	}

	k_delayed_work_submit(&frnd->timer,
			      offer_delay(frnd, rx->rssi, msg->criteria));

	bt_mesh_friend_cred_add(sub->net_idx, sub->keys[0].net, 0,
				frnd->lpn, frnd->lpn_counter, frnd->counter);

	enqueue_offer(frnd, rx->rssi);

	return 0;
}

static struct bt_mesh_friend_seg *get_seg(struct bt_mesh_friend *frnd,
					  u16_t src, u64_t *seq_auth)
{
	struct bt_mesh_friend_seg *unassigned = NULL;
	int i;

	for (i = 0; i < ARRAY_SIZE(frnd->seg); i++) {
		struct bt_mesh_friend_seg *seg = &frnd->seg[i];
		struct os_mbuf *buf = (void *)sys_slist_peek_head(&seg->queue);

		if (buf && BT_MESH_ADV(buf)->addr == src &&
		    buf_seq_auth[net_buf_id(buf)] == *seq_auth) {
			return seg;
		}

		if (!unassigned && !buf) {
			unassigned = seg;
		}
	}

	return unassigned;
}

static void enqueue_friend_pdu(struct bt_mesh_friend *frnd,
			       enum bt_mesh_friend_pdu_type type,
			       struct os_mbuf *buf)
{
	struct bt_mesh_friend_seg *seg;

	BT_DBG("type %u", type);

	if (type == BT_MESH_FRIEND_PDU_SINGLE) {
		if (frnd->sec_update) {
			enqueue_update(frnd, 1);
		}

		enqueue_buf(frnd, buf);
		return;
	}

	seg = get_seg(frnd, BT_MESH_ADV(buf)->addr,
		      &buf_seq_auth[net_buf_id(buf)]);
	if (!seg) {
		BT_ERR("No free friend segment RX contexts for 0x%04x",
		       BT_MESH_ADV(buf)->addr);
		net_buf_unref(buf);
		return;
	}

	net_buf_slist_put(&seg->queue, buf);

	if (type == BT_MESH_FRIEND_PDU_COMPLETE) {
		if (frnd->sec_update) {
			enqueue_update(frnd, 1);
		}

		/* Only acks should have a valid SeqAuth in the Friend queue
		 * (otherwise we can't easily detect them there), so clear
		 * the SeqAuth information from the segments before merging.
		 */
		struct os_mbuf *m;
		struct os_mbuf_pkthdr *mp;
		SYS_SLIST_FOR_EACH_CONTAINER(&seg->queue, mp,) {
			m = OS_MBUF_PKTHDR_TO_MBUF(mp);
			buf_seq_auth[net_buf_id(m)] = TRANS_SEQ_AUTH_NVAL;
			frnd->queue_size++;
		}

		sys_slist_merge_slist(&frnd->queue, &seg->queue);
	}
}

static void buf_sent(struct os_mbuf *buf, u16_t duration, int err)
{
	struct bt_mesh_friend *frnd = NULL;
	int i;

	BT_DBG("buf %p err %d", buf, err);

	for (i = 0; i < ARRAY_SIZE(bt_mesh.frnd); i++) {
		if (bt_mesh.frnd[i].last == buf) {
			frnd = &bt_mesh.frnd[i];
			break;
		}
	}

	if (!frnd) {
		return;
	}

	frnd->pending_buf = 0;

	if (frnd->established) {
		k_delayed_work_submit(&frnd->timer, duration + frnd->poll_to);
		BT_DBG("Waiting %u ms for next poll", frnd->poll_to);
	} else {
		/* Friend offer timeout is 1 second */
		k_delayed_work_submit(&frnd->timer, duration + K_SECONDS(1));
		BT_DBG("Waiting for first poll");

		/* Friend Offer doesn't follow the re-sending semantics */
		net_buf_unref(frnd->last);
		frnd->last = NULL;
	}
}

static void friend_timeout(struct os_event *work)
{
	struct bt_mesh_friend *frnd = work->ev_arg;

	__ASSERT_NO_MSG(frnd->pending_buf == 0);

	BT_DBG("lpn 0x%04x send_last %u last %p", frnd->lpn,
	       frnd->send_last, frnd->last);

	if (frnd->send_last && frnd->last) {
		BT_DBG("Sending frnd->last");
		frnd->send_last = 0;
		goto send_last;
	}

	if (frnd->last) {
		net_buf_unref(frnd->last);
	}

	/* If the timeout is triggered without anything in the queue, it
	 * means that there was no poll/req from the LPN and the timeout
	 * occured because we hit the poll timeout.
	 */
	frnd->last = net_buf_slist_get(&frnd->queue);
	if (!frnd->last) {
		BT_WARN("Friendship %s",
			frnd->established ? "lost" : "not established");
		friend_clear(frnd);
		return;
	}

	BT_DBG("Sending buf %p from Friend Queue of LPN 0x%04x",
	       frnd->last, frnd->lpn);

send_last:
	frnd->pending_buf = 1;
	bt_mesh_adv_send(frnd->last, buf_sent);
}

int bt_mesh_friend_init(void)
{
	int rc;
	int i;

	rc = os_mempool_init(&friend_buf_mempool, MYNEWT_VAL(BLE_MESH_ADV_BUF_COUNT),
			BT_MESH_ADV_DATA_SIZE + BT_MESH_ADV_USER_DATA_SIZE,
			friend_buf_mem, "friend_buf_pool");
	assert(rc == 0);

	rc = os_mbuf_pool_init(&friend_os_mbuf_pool, &friend_buf_mempool,
			BT_MESH_ADV_DATA_SIZE + BT_MESH_ADV_USER_DATA_SIZE,
			MYNEWT_VAL(BLE_MESH_ADV_BUF_COUNT));
	assert(rc == 0);

	for (i = 0; i < ARRAY_SIZE(bt_mesh.frnd); i++) {
		struct bt_mesh_friend *frnd = &bt_mesh.frnd[i];
		int j;

		frnd->net_idx = BT_MESH_KEY_UNUSED;

		sys_slist_init(&frnd->queue);

		k_delayed_work_init(&frnd->timer, friend_timeout);
		k_delayed_work_add_arg(&frnd->timer, frnd);
		k_delayed_work_init(&frnd->clear.timer, clear_timeout);
		k_delayed_work_add_arg(&frnd->clear.timer, frnd);

		for (j = 0; j < ARRAY_SIZE(frnd->seg); j++) {
			sys_slist_init(&frnd->seg[j].queue);
		}
	}

	return 0;
}

static void friend_purge_old_ack(struct bt_mesh_friend *frnd, u64_t *seq_auth,
				 u16_t src)
{
	sys_snode_t *cur, *prev = NULL;

	BT_DBG("SeqAuth %llx src 0x%04x", *seq_auth, src);

	for (cur = sys_slist_peek_head(&frnd->queue);
	     cur != NULL; prev = cur, cur = sys_slist_peek_next(cur)) {
		struct os_mbuf *buf = (void *)cur;

		if (BT_MESH_ADV(buf)->addr == src &&
		    buf_seq_auth[net_buf_id(buf)] == *seq_auth) {
			BT_DBG("Removing old ack from Friend Queue");

			sys_slist_remove(&frnd->queue, prev, cur);

			net_buf_unref(buf);
			break;
		}
	}
}

static void friend_lpn_enqueue_rx(struct bt_mesh_friend *frnd,
				  struct bt_mesh_net_rx *rx,
				  enum bt_mesh_friend_pdu_type type,
				  u64_t *seq_auth, struct os_mbuf *sbuf)
{
	struct friend_pdu_info info;
	struct os_mbuf *buf;

	BT_DBG("LPN 0x%04x", frnd->lpn);

	if (type == BT_MESH_FRIEND_PDU_SINGLE && seq_auth) {
		friend_purge_old_ack(frnd, seq_auth, rx->ctx.addr);
	}

	info.src = rx->ctx.addr;
	info.dst = rx->dst;

	if (rx->net_if == BT_MESH_NET_IF_LOCAL) {
		info.ttl = rx->ctx.recv_ttl;
	} else {
		info.ttl = rx->ctx.recv_ttl - 1;
	}

	info.ctl = rx->ctl;

	info.seq[0] = (rx->seq >> 16);
	info.seq[1] = (rx->seq >> 8);
	info.seq[2] = rx->seq;

	info.iv_index = BT_MESH_NET_IVI_RX(rx);

	buf = create_friend_pdu(frnd, &info, sbuf);
	if (!buf) {
		BT_ERR("Failed to encode Friend buffer");
		return;
	}

	if (seq_auth) {
		buf_seq_auth[net_buf_id(buf)] = *seq_auth;
	}

	enqueue_friend_pdu(frnd, type, buf);

	BT_DBG("Queued message for LPN 0x%04x", frnd->lpn);
}

static void friend_lpn_enqueue_tx(struct bt_mesh_friend *frnd,
				  struct bt_mesh_net_tx *tx,
				  enum bt_mesh_friend_pdu_type type,
				  u64_t *seq_auth, struct os_mbuf *sbuf)
{
	struct friend_pdu_info info;
	struct os_mbuf *buf;

	BT_DBG("LPN 0x%04x", frnd->lpn);

	if (type == BT_MESH_FRIEND_PDU_SINGLE && seq_auth) {
		friend_purge_old_ack(frnd, seq_auth, tx->src);
	}

	info.src = tx->src;
	info.dst = tx->ctx->addr;

	info.ttl = tx->ctx->send_ttl;
	info.ctl = (tx->ctx->app_idx == BT_MESH_KEY_UNUSED);

	info.seq[0] = (bt_mesh.seq >> 16);
	info.seq[1] = (bt_mesh.seq >> 8);
	info.seq[2] = bt_mesh.seq++;

	info.iv_index = BT_MESH_NET_IVI_TX;

	buf = create_friend_pdu(frnd, &info, sbuf);
	if (!buf) {
		BT_ERR("Failed to encode Friend buffer");
		return;
	}

	if (seq_auth) {
		buf_seq_auth[net_buf_id(buf)] = *seq_auth;
	}

	enqueue_friend_pdu(frnd, type, buf);

	BT_DBG("Queued message for LPN 0x%04x", frnd->lpn);
}

static bool friend_lpn_matches(struct bt_mesh_friend *frnd, u16_t net_idx,
			       u16_t addr)
{
	int i;

	BT_DBG("frnd->established %u frnd->lpn 0x%04x frnd->net_idx 0x%04x",
	       frnd->established, frnd->lpn, frnd->net_idx);
	BT_DBG("addr 0x%04x net_idx 0x%04x", addr, net_idx);

	if (!frnd->established) {
		return false;
	}

	if (net_idx != frnd->net_idx) {
		return false;
	}

	if (BT_MESH_ADDR_IS_UNICAST(addr)) {
		if (addr == frnd->lpn) {
			return true;
		}

		return false;
	}

	for (i = 0; i < ARRAY_SIZE(frnd->sub_list); i++) {
		if (frnd->sub_list[i] == addr) {
			return true;
		}
	}

	return false;
}

bool bt_mesh_friend_match(u16_t net_idx, u16_t addr)
{
	int i;

	BT_DBG("net_idx 0x%04x addr 0x%04x", net_idx, addr);

	for (i = 0; i < ARRAY_SIZE(bt_mesh.frnd); i++) {
		struct bt_mesh_friend *frnd = &bt_mesh.frnd[i];

		if (friend_lpn_matches(frnd, net_idx, addr)) {
			return true;
		}
	}

	return false;
}

void bt_mesh_friend_enqueue_rx(struct bt_mesh_net_rx *rx,
			       enum bt_mesh_friend_pdu_type type,
			       u64_t *seq_auth, struct os_mbuf *sbuf)
{
	int i;

	if (!rx->friend_match ||
	    (rx->ctx.recv_ttl <= 1 && rx->net_if != BT_MESH_NET_IF_LOCAL) ||
	    bt_mesh_friend_get() != BT_MESH_FRIEND_ENABLED) {
		return;
	}

	BT_DBG("recv_ttl %u net_idx 0x%04x src 0x%04x dst 0x%04x",
	       rx->ctx.recv_ttl, rx->sub->net_idx, rx->ctx.addr, rx->dst);

	for (i = 0; i < ARRAY_SIZE(bt_mesh.frnd); i++) {
		struct bt_mesh_friend *frnd = &bt_mesh.frnd[i];

		if (friend_lpn_matches(frnd, rx->sub->net_idx, rx->dst)) {
			friend_lpn_enqueue_rx(frnd, rx, type, seq_auth, sbuf);
		}
	}
}

bool bt_mesh_friend_enqueue_tx(struct bt_mesh_net_tx *tx,
			       enum bt_mesh_friend_pdu_type type,
			       u64_t *seq_auth, struct os_mbuf *sbuf)
{
	bool matched = false;
	int i;

	if (!bt_mesh_friend_match(tx->sub->net_idx, tx->ctx->addr) ||
	    bt_mesh_friend_get() != BT_MESH_FRIEND_ENABLED) {
		return matched;
	}

	BT_DBG("net_idx 0x%04x dst 0x%04x src 0x%04x", tx->sub->net_idx,
	       tx->ctx->addr, tx->src);

	for (i = 0; i < ARRAY_SIZE(bt_mesh.frnd); i++) {
		struct bt_mesh_friend *frnd = &bt_mesh.frnd[i];

		if (friend_lpn_matches(frnd, tx->sub->net_idx, tx->ctx->addr)) {
			friend_lpn_enqueue_tx(frnd, tx, type, seq_auth, sbuf);
			matched = true;
		}
	}

	return matched;
}

void bt_mesh_friend_clear_incomplete(struct bt_mesh_subnet *sub, u16_t src,
				     u16_t dst, u64_t *seq_auth)
{
	int i;

	BT_DBG("");

	for (i = 0; i < ARRAY_SIZE(bt_mesh.frnd); i++) {
		struct bt_mesh_friend *frnd = &bt_mesh.frnd[i];
		int j;

		if (!friend_lpn_matches(frnd, sub->net_idx, dst)) {
			continue;
		}

		for (j = 0; j < ARRAY_SIZE(frnd->seg); j++) {
			struct bt_mesh_friend_seg *seg = &frnd->seg[j];
			struct os_mbuf *buf;

			buf = (void *)sys_slist_peek_head(&seg->queue);
			if (!buf) {
				continue;
			}

			if (BT_MESH_ADV(buf)->addr != src) {
				continue;
			}

			if (buf_seq_auth[net_buf_id(buf)] != *seq_auth) {
				continue;
			}

			BT_WARN("Clearing incomplete segments for 0x%04x", src);

			while (!sys_slist_is_empty(&seg->queue)) {
				net_buf_unref(net_buf_slist_get(&seg->queue));
			}
		}
	}
}

#endif //MYNEWT_VAL(BLE_MESH_FRIEND)
