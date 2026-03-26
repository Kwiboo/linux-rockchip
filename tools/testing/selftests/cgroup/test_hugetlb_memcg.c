// SPDX-License-Identifier: GPL-2.0
#define _GNU_SOURCE

#include <linux/limits.h>
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include "kselftest.h"
#include "cgroup_util.h"

#define ADDR ((void *)(0x0UL))
#define FLAGS (MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB)
#define PROTECTION (PROT_READ | PROT_WRITE)

/*
 * This value matches the kernel's MEMCG_CHARGE_BATCH definition:
 * see include/linux/memcontrol.h. If the kernel value changes, this
 * test constant must be updated accordingly to stay consistent.
 */
#define MEMCG_CHARGE_BATCH 64U

/* borrowed from mm/hmm-tests.c */
static long get_hugepage_size(void)
{
	int fd;
	char buf[2048];
	int len;
	char *p, *q, *path = "/proc/meminfo", *tag = "Hugepagesize:";
	long val;

	fd = open(path, O_RDONLY);
	if (fd < 0) {
		/* Error opening the file */
		return -1;
	}

	len = read(fd, buf, sizeof(buf));
	close(fd);
	if (len < 0) {
		/* Error in reading the file */
		return -1;
	}
	if (len == sizeof(buf)) {
		/* Error file is too large */
		return -1;
	}
	buf[len] = '\0';

	/* Search for a tag if provided */
	if (tag) {
		p = strstr(buf, tag);
		if (!p)
			return -1; /* looks like the line we want isn't there */
		p += strlen(tag);
	} else
		p = buf;

	val = strtol(p, &q, 0);
	if (*q != ' ') {
		/* Error parsing the file */
		return -1;
	}

	return val;
}

static int set_file(const char *path, long value)
{
	FILE *file;
	int ret;

	file = fopen(path, "w");
	if (!file)
		return -1;
	ret = fprintf(file, "%ld\n", value);
	fclose(file);
	return ret;
}

static int set_nr_hugepages(long value)
{
	return set_file("/proc/sys/vm/nr_hugepages", value);
}

static unsigned int check_first(char *addr)
{
	return *(unsigned int *)addr;
}

static void write_data(char *addr, size_t length)
{
	unsigned long i;

	for (i = 0; i < length; i++)
		*(addr + i) = (char)i;
}

static int hugetlb_test_program(const char *cgroup, void *arg)
{
	char *test_group = (char *)arg;
	void *addr;
	long hpage_size = get_hugepage_size() * 1024;
	long old_current, expected_current, current;
	int ret = EXIT_FAILURE;
	size_t length = 4 * hpage_size;
	int pagesize, nr_pages;

	pagesize = getpagesize();

	old_current = cg_read_long(test_group, "memory.current");
	set_nr_hugepages(20);
	current = cg_read_long(test_group, "memory.current");
	if (current - old_current >= hpage_size) {
		ksft_print_msg(
			"setting nr_hugepages should not increase hugepage usage.\n");
		ksft_print_msg("before: %ld, after: %ld\n", old_current, current);
		return EXIT_FAILURE;
	}

	addr = mmap(ADDR, length, PROTECTION, FLAGS, 0, 0);
	if (addr == MAP_FAILED) {
		ksft_print_msg("fail to mmap.\n");
		return EXIT_FAILURE;
	}
	current = cg_read_long(test_group, "memory.current");
	if (current - old_current >= hpage_size) {
		ksft_print_msg("mmap should not increase hugepage usage.\n");
		ksft_print_msg("before: %ld, after: %ld\n", old_current, current);
		goto out_failed_munmap;
	}
	old_current = current;

	/* read the first page */
	check_first(addr);
	nr_pages = hpage_size / pagesize;
	expected_current = old_current + hpage_size;
	current = cg_read_long(test_group, "memory.current");
	if (nr_pages < MEMCG_CHARGE_BATCH && current == old_current) {
		/*
		 * Memory cgroup charging uses per-CPU stocks and batched updates to the
		 *  memcg usage counters. For hugetlb allocations, the number of pages
		 *  that memcg charges is expressed in base pages (nr_pages), not
		 *  in hugepage units. When the charge for an allocation is smaller than
		 *  the internal batching threshold  (nr_pages <  MEMCG_CHARGE_BATCH),
		 *  it may be fully satisfied from the CPU’s local stock. In such
		 *  cases memory.current does not necessarily
		 *  increase.
		 *  Therefore, Treat a zero delta as valid behaviour here.
		 */
		ksft_print_msg("no visible memcg charge, allocation consumed from local stock.\n");
	} else if (!values_close(expected_current, current, 5)) {
		ksft_print_msg("memory usage should increase by ~1 huge page.\n");
		ksft_print_msg(
			"expected memory: %ld, actual memory: %ld\n",
			expected_current, current);
		goto out_failed_munmap;
	}

	/* write to the whole range */
	write_data(addr, length);
	current = cg_read_long(test_group, "memory.current");
	expected_current = old_current + length;
	if (!values_close(expected_current, current, 5)) {
		ksft_print_msg("memory usage should increase by around 4 huge pages.\n");
		ksft_print_msg(
			"expected memory: %ld, actual memory: %ld\n",
			expected_current, current);
		goto out_failed_munmap;
	}

	/* unmap the whole range */
	munmap(addr, length);
	current = cg_read_long(test_group, "memory.current");
	expected_current = old_current;
	if (!values_close(expected_current, current, 5)) {
		ksft_print_msg("memory usage should go back down.\n");
		ksft_print_msg(
			"expected memory: %ld, actual memory: %ld\n",
			expected_current, current);
		return ret;
	}

	ret = EXIT_SUCCESS;
	return ret;

out_failed_munmap:
	munmap(addr, length);
	return ret;
}

static int test_hugetlb_memcg(char *root)
{
	int ret = KSFT_FAIL;
	int num_pages = 20;
	long hpage_size = get_hugepage_size();
	char *test_group;

	test_group = cg_name(root, "hugetlb_memcg_test");
	if (!test_group || cg_create(test_group)) {
		ksft_print_msg("fail to create cgroup.\n");
		goto out;
	}

	if (cg_write_numeric(test_group, "memory.max", num_pages * hpage_size * 1024)) {
		ksft_print_msg("fail to set cgroup memory limit.\n");
		goto out;
	}

	/* disable swap */
	if (cg_write(test_group, "memory.swap.max", "0")) {
		ksft_print_msg("fail to disable swap.\n");
		goto out;
	}

	if (!cg_run(test_group, hugetlb_test_program, (void *)test_group))
		ret = KSFT_PASS;
out:
	cg_destroy(test_group);
	free(test_group);
	return ret;
}

int main(int argc, char **argv)
{
	char root[PATH_MAX];
	int ret = EXIT_SUCCESS, has_memory_hugetlb_acc;
	long val;

	has_memory_hugetlb_acc = proc_mount_contains("memory_hugetlb_accounting");
	if (has_memory_hugetlb_acc < 0)
		ksft_exit_skip("Failed to query cgroup mount option\n");
	else if (!has_memory_hugetlb_acc)
		ksft_exit_skip("memory hugetlb accounting is disabled\n");

	/* Unit is kB! */
	val = get_hugepage_size();
	if (val < 0) {
		ksft_print_msg("Failed to read hugepage size\n");
		ksft_test_result_skip("test_hugetlb_memcg\n");
		return ret;
	}

	ksft_print_msg("Hugepage size: %ld kB\n", val);

	if (cg_find_unified_root(root, sizeof(root), NULL))
		ksft_exit_skip("cgroup v2 isn't mounted\n");

	switch (test_hugetlb_memcg(root)) {
	case KSFT_PASS:
		ksft_test_result_pass("test_hugetlb_memcg\n");
		break;
	case KSFT_SKIP:
		ksft_test_result_skip("test_hugetlb_memcg\n");
		break;
	default:
		ret = EXIT_FAILURE;
		ksft_test_result_fail("test_hugetlb_memcg\n");
		break;
	}

	return ret;
}
