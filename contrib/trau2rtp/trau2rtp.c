#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <osmocom/core/utils.h>
#include <osmocom/core/application.h>
#include <osmocom/gsm/i460_mux.h>
#include <osmocom/trau/trau_sync.h>
#include <osmocom/trau/trau_frame.h>
#include <osmocom/trau/trau_rtp.h>
#include <osmocom/trau/osmo_ortp.h>

#include "flip.h"

const char *g_remote_host = "127.0.0.1";
const int g_remote_port = 9999;

struct state {
	int in_fd;
	struct osmo_i460_timeslot i460_ts;
	struct osmo_fsm_inst *sync_fi;
	struct osmo_rtp_socket *rtps;
};
static struct state g_st;


/* called by I.460 de-multeiplexer; feed output of I.460 demux into TRAU frame sync */
static void i460_demux_bits_cb(void *user_data, const ubit_t *bits, unsigned int num_bits)
{
	struct state *st = user_data;
	//printf("I460: %s\n", osmo_ubit_dump(bits, num_bits));
	osmo_trau_sync_rx_ubits(st->sync_fi, bits, num_bits);
}

/* called for each synchronized TRAU frame received; decode frame + convert to RTP */
static void sync_frame_out_cb(void *user_data, const ubit_t *bits, unsigned int num_bits)
{
	struct state *st = user_data;
	struct osmo_trau_frame fr;
	int rc;

	printf("TRAU: %s\n", osmo_ubit_dump(bits, num_bits));
	if (!bits)
		goto skip;

	rc = osmo_trau_frame_decode_16k(&fr, bits, OSMO_TRAU_DIR_UL);
	if (rc != 0)
		goto skip;

	printf("-> FT=%s\n", osmo_trau_frame_type_name(fr.type));
	if (fr.type != OSMO_TRAU16_FT_FR && fr.type != OSMO_TRAU16_FT_EFR)
		goto skip;

	uint8_t rtpbuf[35];
	struct osmo_trau2rtp_state t2rs = {
		.type = fr.type,
	};
	memset(rtpbuf, 0, sizeof(rtpbuf));
	rc = osmo_trau2rtp(rtpbuf, sizeof(rtpbuf), &fr, &t2rs);
	printf("RTP: %s\n", osmo_hexdump(rtpbuf, rc));
	if (rc)
		osmo_rtp_send_frame_ext(st->rtps, rtpbuf, rc, 160, false);
	else {
skip:
		osmo_rtp_skipped_frame(st->rtps, 160);
	}
}

static void init(const char *fname)
{
	struct osmo_i460_schan_desc scd16_0 = {
		.rate = OSMO_I460_RATE_16k,
		.bit_offset = 2,
		.demux = {
			.num_bits = 40*8,
			.out_cb_bits = i460_demux_bits_cb,
			.out_cb_bytes = NULL,
			.user_data = &g_st,
		},
	};

	g_st.in_fd = open(fname, O_RDONLY);
	OSMO_ASSERT(g_st.in_fd >= 0);

	osmo_i460_ts_init(&g_st.i460_ts);
	OSMO_ASSERT(osmo_i460_subchan_add(NULL, &g_st.i460_ts, &scd16_0) != NULL);

	g_st.sync_fi = osmo_trau_sync_alloc(NULL, "test", sync_frame_out_cb, &g_st);
	g_st.rtps = osmo_rtp_socket_create(NULL, 0);
	osmo_rtp_socket_set_pt(g_st.rtps, RTP_PT_GSM_FULL);
	//osmo_rtp_socket_bind(g_st.rtps, "localhost", );
	osmo_rtp_socket_connect(g_st.rtps, g_remote_host, g_remote_port);
}


static void process(void)
{
	uint8_t buf[123];
	int rc;

	while (rc = read(g_st.in_fd, buf, sizeof(buf))) {
		flip_buf_bits(buf, rc);
		osmo_i460_demux_in(&g_st.i460_ts, buf, rc);
		//usleep(20000);
	}
}

int main(int argc, char **argv)
{
	osmo_init_logging2(NULL, NULL);
	osmo_fsm_log_addr(false);
	log_set_print_filename2(osmo_stderr_target, LOG_FILENAME_BASENAME);
	osmo_rtp_init(NULL);
	init_flip_bits();

	init(argv[1]);
	process();
}
