#include <common.h>
#include <command.h>
#include <jffs2/jffs2.h>

#include "mtd.h"

static char kobsng_help_text[] =
	"kobsng - addr off\n"
	"";

static int do_kobs_ng(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	ulong addr;
	int r;
	loff_t off, size, maxsize;
	struct mtd_data *md;
	struct mtd_config cfg;
	int dev = nand_curr_device;
	int flags = 0;

 	flags |= F_VERBOSE;

	addr = (ulong)simple_strtoul(argv[1], NULL, 16);

	mtd_arg_off_size(argc - 2, argv + 2, &dev, &off,
			&size, &maxsize,
			MTD_DEV_TYPE_NAND,
			nand_info[dev].size);
	memcpy(&cfg, &default_mtd_config, sizeof(cfg));

	md = mtd_open(&cfg, flags);
	if (md == NULL) {
		printf("Unable to open mtd device(s)\n");
		return -1;
	}

	r = v4_rom_mtd_init(md, size);
	if (r < 0) {
		printf("mtd_init failed!\n");
		return r;
	}

	r = v6_rom_mtd_commit_structures(md, addr);
	if (r < 0) {
		printf("FAILED to commit structures\n");
		return r;
	}

	printf("ready now..\n");

	mtd_close(md);

	

	return 0;
}

U_BOOT_CMD(
	kobsng, 4, 0,	do_kobs_ng,
	"NAND sub-system", kobsng_help_text
);
