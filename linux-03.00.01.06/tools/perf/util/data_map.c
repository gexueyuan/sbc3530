#include "data_map.h"
#include "symbol.h"
#include "util.h"
#include "debug.h"


static struct perf_file_handler *curr_handler;
static unsigned long	mmap_window = 32;
static char		__cwd[PATH_MAX];

static int process_event_stub(event_t *event __used)
{
	dump_printf(": unhandled!\n");
	return 0;
}

void register_perf_file_handler(struct perf_file_handler *handler)
{
	if (!handler->process_sample_event)
		handler->process_sample_event = process_event_stub;
	if (!handler->process_mmap_event)
		handler->process_mmap_event = process_event_stub;
	if (!handler->process_comm_event)
		handler->process_comm_event = process_event_stub;
	if (!handler->process_fork_event)
		handler->process_fork_event = process_event_stub;
	if (!handler->process_exit_event)
		handler->process_exit_event = process_event_stub;
	if (!handler->process_lost_event)
		handler->process_lost_event = process_event_stub;
	if (!handler->process_read_event)
		handler->process_read_event = process_event_stub;
	if (!handler->process_throttle_event)
		handler->process_throttle_event = process_event_stub;
	if (!handler->process_unthrottle_event)
		handler->process_unthrottle_event = process_event_stub;

	curr_handler = handler;
}

static const char *event__name[] = {
	[0]			 = "TOTAL",
	[PERF_RECORD_MMAP]	 = "MMAP",
	[PERF_RECORD_LOST]	 = "LOST",
	[PERF_RECORD_COMM]	 = "COMM",
	[PERF_RECORD_EXIT]	 = "EXIT",
	[PERF_RECORD_THROTTLE]	 = "THROTTLE",
	[PERF_RECORD_UNTHROTTLE] = "UNTHROTTLE",
	[PERF_RECORD_FORK]	 = "FORK",
	[PERF_RECORD_READ]	 = "READ",
	[PERF_RECORD_SAMPLE]	 = "SAMPLE",
};

unsigned long event__total[PERF_RECORD_MAX];

void event__print_totals(void)
{
	int i;
	for (i = 0; i < PERF_RECORD_MAX; ++i)
		pr_info("%10s events: %10ld\n",
			event__name[i], event__total[i]);
}

static int
process_event(event_t *event, unsigned long offset, unsigned long head)
{
	trace_event(event);

	if (event->header.type < PERF_RECORD_MAX) {
		dump_printf("%p [%p]: PERF_RECORD_%s",
			    (void *)(offset + head),
			    (void *)(long)(event->header.size),
			    event__name[event->header.type]);
		++event__total[0];
		++event__total[event->header.type];
	}

	switch (event->header.type) {
	case PERF_RECORD_SAMPLE:
		return curr_handler->process_sample_event(event);
	case PERF_RECORD_MMAP:
		return curr_handler->process_mmap_event(event);
	case PERF_RECORD_COMM:
		return curr_handler->process_comm_event(event);
	case PERF_RECORD_FORK:
		return curr_handler->process_fork_event(event);
	case PERF_RECORD_EXIT:
		return curr_handler->process_exit_event(event);
	case PERF_RECORD_LOST:
		return curr_handler->process_lost_event(event);
	case PERF_RECORD_READ:
		return curr_handler->process_read_event(event);
	case PERF_RECORD_THROTTLE:
		return curr_handler->process_throttle_event(event);
	case PERF_RECORD_UNTHROTTLE:
		return curr_handler->process_unthrottle_event(event);
	default:
		curr_handler->total_unknown++;
		return -1;
	}
}

int perf_header__read_build_ids(int input, u64 offset, u64 size)
{
	struct build_id_event bev;
	char filename[PATH_MAX];
	u64 limit = offset + size;
	int err = -1;

	while (offset < limit) {
		struct dso *dso;
		ssize_t len;

		if (read(input, &bev, sizeof(bev)) != sizeof(bev))
			goto out;

		len = bev.header.size - sizeof(bev);
		if (read(input, filename, len) != len)
			goto out;

		dso = dsos__findnew(filename);
		if (dso != NULL)
			dso__set_build_id(dso, &bev.build_id);

		offset += bev.header.size;
	}
	err = 0;
out:
	return err;
}

int perf_session__process_events(struct perf_session *self,
				 int full_paths, int *cwdlen, char **cwd)
{
	int err;
	unsigned long head, shift;
	unsigned long offset = 0;
	size_t	page_size;
	u64 sample_type;
	event_t *event;
	uint32_t size;
	char *buf;

	if (curr_handler == NULL) {
		pr_debug("Forgot to register perf file handler\n");
		return -EINVAL;
	}

	page_size = getpagesize();

	head = self->header.data_offset;
	sample_type = perf_header__sample_type(&self->header);

	err = -EINVAL;
	if (curr_handler->sample_type_check &&
	    curr_handler->sample_type_check(sample_type) < 0)
		goto out_err;

	if (!full_paths) {
		if (getcwd(__cwd, sizeof(__cwd)) == NULL) {
			pr_err("failed to get the current directory\n");
			err = -errno;
			goto out_err;
		}
		*cwd = __cwd;
		*cwdlen = strlen(*cwd);
	} else {
		*cwd = NULL;
		*cwdlen = 0;
	}

	shift = page_size * (head / page_size);
	offset += shift;
	head -= shift;

remap:
	buf = mmap(NULL, page_size * mmap_window, PROT_READ,
		   MAP_SHARED, self->fd, offset);
	if (buf == MAP_FAILED) {
		pr_err("failed to mmap file\n");
		err = -errno;
		goto out_err;
	}

more:
	event = (event_t *)(buf + head);

	size = event->header.size;
	if (!size)
		size = 8;

	if (head + event->header.size >= page_size * mmap_window) {
		int munmap_ret;

		shift = page_size * (head / page_size);

		munmap_ret = munmap(buf, page_size * mmap_window);
		assert(munmap_ret == 0);

		offset += shift;
		head -= shift;
		goto remap;
	}

	size = event->header.size;

	dump_printf("\n%p [%p]: event: %d\n",
			(void *)(offset + head),
			(void *)(long)event->header.size,
			event->header.type);

	if (!size || process_event(event, offset, head) < 0) {

		dump_printf("%p [%p]: skipping unknown header type: %d\n",
			(void *)(offset + head),
			(void *)(long)(event->header.size),
			event->header.type);

		/*
		 * assume we lost track of the stream, check alignment, and
		 * increment a single u64 in the hope to catch on again 'soon'.
		 */

		if (unlikely(head & 7))
			head &= ~7ULL;

		size = 8;
	}

	head += size;

	if (offset + head >= self->header.data_offset + self->header.data_size)
		goto done;

	if (offset + head < self->size)
		goto more;

done:
	err = 0;
out_err:
	return err;
}
