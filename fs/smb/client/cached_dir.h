/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  Functions to handle the cached directory entries
 *
 *  Copyright (c) 2022, Ronnie Sahlberg <lsahlber@redhat.com>
 */

#ifndef _CACHED_DIR_H
#define _CACHED_DIR_H

struct cached_dirent {
	struct list_head entry;
	char *name;
	int namelen;
	loff_t pos;
	struct cifs_fattr fattr;
};

struct cached_dirents {
	bool is_valid:1;
	bool is_failed:1;
	struct file *file; /*
			    * Used to associate the cache with a single
			    * open file instance.
			    */
	struct mutex de_mutex;
	loff_t pos;		 /* Expected ctx->pos */
	struct list_head entries;
	/* accounting for cached entries in this directory */
	unsigned long entries_count;
	unsigned long bytes_used;
};

struct cached_fid {
	struct list_head entry;
	struct cached_fids *cfids;
	const char *path;
	bool has_lease;
	bool is_open;
	bool on_list;
	bool file_all_info_is_valid;
	unsigned long time; /* jiffies of when lease was taken */
	unsigned long last_access_time; /* jiffies of when last accessed */
	struct kref refcount;
	struct cifs_fid fid;
	struct cifs_tcon *tcon;
	struct dentry *dentry;
	struct work_struct put_work;
	struct work_struct close_work;
	struct cached_dirents dirents;
	/* Serializes OPEN response processing and lease key population */
	struct mutex cfid_open_mutex;
	spinlock_t cfid_lock;

	/* Must be last as it ends in a flexible-array member. */
	struct smb2_file_all_info file_all_info;
};

/* default MAX_CACHED_FIDS is 16 */
struct cached_fids {
	/* Must be held when:
	 * - modifying cfids->entries list (add/remove entries)
	 * - modifying cfids->dying list
	 * - modifying cfid->on_list or cfids->num_entries
	 *
	 * Lock ordering: if you need both cfid_list_lock and cfid_lock,
	 * acquire cfid_list_lock FIRST, then cfid_lock to avoid deadlock.
	 */
	spinlock_t cfid_list_lock;
	int num_entries;
	struct list_head entries;
	struct list_head dying;
	struct delayed_work laundromat_work;
	/* aggregate accounting for all cached dirents under this tcon */
	atomic_long_t total_dirents_entries;
	atomic64_t total_dirents_bytes;
};

/* Module-wide directory cache accounting (defined in cifsfs.c) */
extern atomic64_t cifs_dircache_bytes_used; /* bytes across all mounts */

static inline bool
is_valid_cached_dir(struct cached_fid *cfid)
{
	return cfid->time && cfid->has_lease;
}

bool cached_dir_copy_lease_key(struct cached_fid *cfid,
			      __u8 lease_key[SMB2_LEASE_KEY_SIZE]);

struct cached_fids *init_cached_dirs(void);
void free_cached_dirs(struct cached_fids *cfids);
int open_cached_dir(unsigned int xid, struct cifs_tcon *tcon, const char *path,
		    struct cifs_sb_info *cifs_sb, bool lookup_only,
		    struct cached_fid **ret_cfid);
int open_cached_dir_by_dentry(struct cifs_tcon *tcon, struct dentry *dentry,
			      struct cached_fid **ret_cfid);
void close_cached_dir(struct cached_fid *cfid);
bool emit_cached_dir_if_valid(struct cached_fid *cfid,
			      struct file *file,
			      struct dir_context *ctx);
bool add_to_cached_dir(struct cached_fid *cfid,
		       struct dir_context *ctx,
		       const char *name,
		       int namelen,
		       struct cifs_fattr *fattr,
		       struct file *file);
void update_pos_cached_dir(struct cached_fid *cfid,
				      struct file *file);
void complete_cached_dir(struct cached_fid *cfid,
					struct dir_context *ctx,
					struct file *file);
struct cached_dirent *lookup_cached_dirent(struct cached_dirents *cde,
				   const char *name,
				   unsigned int namelen);
void drop_cached_dir_by_name(const unsigned int xid, struct cifs_tcon *tcon,
			     const char *name, struct cifs_sb_info *cifs_sb);
void close_all_cached_dirs(struct cifs_sb_info *cifs_sb);
void invalidate_all_cached_dirs(struct cifs_tcon *tcon, bool sync);
bool cached_dir_lease_break(struct cifs_tcon *tcon, __u8 lease_key[16]);

#endif			/* _CACHED_DIR_H */
