#include <linux/types.h>
#include "event.h"
#include "debug.h"
#include "string.h"
#include "thread.h"

static pid_t event__synthesize_comm(pid_t pid, int full,
				    int (*process)(event_t *event))
{
	event_t ev;
	char filename[PATH_MAX];
	char bf[BUFSIZ];
	FILE *fp;
	size_t size = 0;
	DIR *tasks;
	struct dirent dirent, *next;
	pid_t tgid = 0;

	snprintf(filename, sizeof(filename), "/proc/%d/status", pid);

	fp = fopen(filename, "r");
	if (fp == NULL) {
out_race:
		/*
		 * We raced with a task exiting - just return:
		 */
		pr_debug("couldn't open %s\n", filename);
		return 0;
	}

	memset(&ev.comm, 0, sizeof(ev.comm));
	while (!ev.comm.comm[0] || !ev.comm.pid) {
		if (fgets(bf, sizeof(bf), fp) == NULL)
			goto out_failure;

		if (memcmp(bf, "Name:", 5) == 0) {
			char *name = bf + 5;
			while (*name && isspace(*name))
				++name;
			size = strlen(name) - 1;
			memcpy(ev.comm.comm, name, size++);
		} else if (memcmp(bf, "Tgid:", 5) == 0) {
			char *tgids = bf + 5;
			while (*tgids && isspace(*tgids))
				++tgids;
			tgid = ev.comm.pid = atoi(tgids);
		}
	}

	ev.comm.header.type = PERF_RECORD_COMM;
	size = ALIGN(size, sizeof(u64));
	ev.comm.header.size = sizeof(ev.comm) - (sizeof(ev.comm.comm) - size);

	if (!full) {
		ev.comm.tid = pid;

		process(&ev);
		goto out_fclose;
	}

	snprintf(filename, sizeof(filename), "/proc/%d/task", pid);

	tasks = opendir(filename);
	if (tasks == NULL)
		goto out_race;

	while (!readdir_r(tasks, &dirent, &next) && next) {
		char *end;
		pid = strtol(dirent.d_name, &end, 10);
		if (*end)
			continue;

		ev.comm.tid = pid;

		process(&ev);
	}
	closedir(tasks);

out_fclose:
	fclose(fp);
	return tgid;

out_failure:
	pr_warning("couldn't get COMM and pgid, malformed %s\n", filename);
	return -1;
}

static int event__synthesize_mmap_events(pid_t pid, pid_t tgid,
					 int (*process)(event_t *event))
{
	char filename[PATH_MAX];
	FILE *fp;

	snprintf(filename, sizeof(filename), "/proc/%d/maps", pid);

	fp = fopen(filename, "r");
	if (fp == NULL) {
		/*
		 * We raced with a task exiting - just return:
		 */
		pr_debug("couldn't open %s\n", filename);
		return -1;
	}

	while (1) {
		char bf[BUFSIZ], *pbf = bf;
		event_t ev = {
			.header = { .type = PERF_RECORD_MMAP },
		};
		int n;
		size_t size;
		if (fgets(bf, sizeof(bf), fp) == NULL)
			break;

		/* 00400000-0040c000 r-xp 00000000 fd:01 41038  /bin/cat */
		n = hex2u64(pbf, &ev.mmap.start);
		if (n < 0)
			continue;
		pbf += n + 1;
		n = hex2u64(pbf, &ev.mmap.len);
		if (n < 0)
			continue;
		pbf += n + 3;
		if (*pbf == 'x') { /* vm_exec */
			char *execname = strchr(bf, '/');

			/* Catch VDSO */
			if (execname == NULL)
				execname = strstr(bf, "[vdso]");

			if (execname == NULL)
				continue;

			size = strlen(execname);
			execname[size - 1] = '\0'; /* Remove \n */
			memcpy(ev.mmap.filename, execname, size);
			size = ALIGN(size, sizeof(u64));
			ev.mmap.len -= ev.mmap.start;
			ev.mmap.header.size = (sizeof(ev.mmap) -
					       (sizeof(ev.mmap.filename) - size));
			ev.mmap.pid = tgid;
			ev.mmap.tid = pid;

			process(&ev);
		}
	}

	fclose(fp);
	return 0;
}

int event__synthesize_thread(pid_t pid, int (*process)(event_t *event))
{
	pid_t tgid = event__synthesize_comm(pid, 1, process);
	if (tgid == -1)
		return -1;
	return event__synthesize_mmap_events(pid, tgid, process);
}

void event__synthesize_threads(int (*process)(event_t *event))
{
	DIR *proc;
	struct dirent dirent, *next;

	proc = opendir("/proc");

	while (!readdir_r(proc, &dirent, &next) && next) {
		char *end;
		pid_t pid = strtol(dirent.d_name, &end, 10);

		if (*end) /* only interested in proper numerical dirents */
			continue;

		event__synthesize_thread(pid, process);
	}

	closedir(proc);
}

char *event__cwd;
int  event__cwdlen;

struct events_stats event__stats;

int event__process_comm(event_t *self)
{
	struct thread *thread = threads__findnew(self->comm.pid);

	dump_printf(": %s:%d\n", self->comm.comm, self->comm.pid);

	if (thread == NULL || thread__set_comm(thread, self->comm.comm)) {
		dump_printf("problem processing PERF_RECORD_COMM, skipping event.\n");
		return -1;
	}

	return 0;
}

int event__process_lost(event_t *self)
{
	dump_printf(": id:%Ld: lost:%Ld\n", self->lost.id, self->lost.lost);
	event__stats.lost += self->lost.lost;
	return 0;
}

int event__process_mmap(event_t *self)
{
	struct thread *thread = threads__findnew(self->mmap.pid);
	struct map *map = map__new(&self->mmap, MAP__FUNCTION,
				   event__cwd, event__cwdlen);

	dump_printf(" %d/%d: [%p(%p) @ %p]: %s\n",
		    self->mmap.pid, self->mmap.tid,
		    (void *)(long)self->mmap.start,
		    (void *)(long)self->mmap.len,
		    (void *)(long)self->mmap.pgoff,
		    self->mmap.filename);

	if (thread == NULL || map == NULL)
		dump_printf("problem processing PERF_RECORD_MMAP, skipping event.\n");
	else
		thread__insert_map(thread, map);

	return 0;
}

int event__process_task(event_t *self)
{
	struct thread *thread = threads__findnew(self->fork.pid);
	struct thread *parent = threads__findnew(self->fork.ppid);

	dump_printf("(%d:%d):(%d:%d)\n", self->fork.pid, self->fork.tid,
		    self->fork.ppid, self->fork.ptid);
	/*
	 * A thread clone will have the same PID for both parent and child.
	 */
	if (thread == parent)
		return 0;

	if (self->header.type == PERF_RECORD_EXIT)
		return 0;

	if (thread == NULL || parent == NULL ||
	    thread__fork(thread, parent) < 0) {
		dump_printf("problem processing PERF_RECORD_FORK, skipping event.\n");
		return -1;
	}

	return 0;
}

void thread__find_addr_location(struct thread *self, u8 cpumode,
				enum map_type type, u64 addr,
				struct addr_location *al,
				symbol_filter_t filter)
{
	struct map_groups *mg = &self->mg;

	al->thread = self;
	al->addr = addr;

	if (cpumode & PERF_RECORD_MISC_KERNEL) {
		al->level = 'k';
		mg = kmaps;
	} else if (cpumode & PERF_RECORD_MISC_USER)
		al->level = '.';
	else {
		al->level = 'H';
		al->map = NULL;
		al->sym = NULL;
		return;
	}
try_again:
	al->map = map_groups__find(mg, type, al->addr);
	if (al->map == NULL) {
		/*
		 * If this is outside of all known maps, and is a negative
		 * address, try to look it up in the kernel dso, as it might be
		 * a vsyscall or vdso (which executes in user-mode).
		 *
		 * XXX This is nasty, we should have a symbol list in the
		 * "[vdso]" dso, but for now lets use the old trick of looking
		 * in the whole kernel symbol list.
		 */
		if ((long long)al->addr < 0 && mg != kmaps) {
			mg = kmaps;
			goto try_again;
		}
		al->sym = NULL;
	} else {
		al->addr = al->map->map_ip(al->map, al->addr);
		al->sym = map__find_symbol(al->map, al->addr, filter);
	}
}

int event__preprocess_sample(const event_t *self, struct addr_location *al,
			     symbol_filter_t filter)
{
	u8 cpumode = self->header.misc & PERF_RECORD_MISC_CPUMODE_MASK;
	struct thread *thread = threads__findnew(self->ip.pid);

	if (thread == NULL)
		return -1;

	dump_printf(" ... thread: %s:%d\n", thread->comm, thread->pid);

	thread__find_addr_location(thread, cpumode, MAP__FUNCTION,
				   self->ip.ip, al, filter);
	dump_printf(" ...... dso: %s\n",
		    al->map ? al->map->dso->long_name :
			al->level == 'H' ? "[hypervisor]" : "<not found>");
	return 0;
}

int event__parse_sample(event_t *event, u64 type, struct sample_data *data)
{
	u64 *array = event->sample.array;

	if (type & PERF_SAMPLE_IP) {
		data->ip = event->ip.ip;
		array++;
	}

	if (type & PERF_SAMPLE_TID) {
		u32 *p = (u32 *)array;
		data->pid = p[0];
		data->tid = p[1];
		array++;
	}

	if (type & PERF_SAMPLE_TIME) {
		data->time = *array;
		array++;
	}

	if (type & PERF_SAMPLE_ADDR) {
		data->addr = *array;
		array++;
	}

	if (type & PERF_SAMPLE_ID) {
		data->id = *array;
		array++;
	}

	if (type & PERF_SAMPLE_STREAM_ID) {
		data->stream_id = *array;
		array++;
	}

	if (type & PERF_SAMPLE_CPU) {
		u32 *p = (u32 *)array;
		data->cpu = *p;
		array++;
	}

	if (type & PERF_SAMPLE_PERIOD) {
		data->period = *array;
		array++;
	}

	if (type & PERF_SAMPLE_READ) {
		pr_debug("PERF_SAMPLE_READ is unsuported for now\n");
		return -1;
	}

	if (type & PERF_SAMPLE_CALLCHAIN) {
		data->callchain = (struct ip_callchain *)array;
		array += 1 + data->callchain->nr;
	}

	if (type & PERF_SAMPLE_RAW) {
		u32 *p = (u32 *)array;
		data->raw_size = *p;
		p++;
		data->raw_data = p;
	}

	return 0;
}
