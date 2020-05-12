/*
 *  mtd.c - Dump control structures of the NAND
 *
 *  Copyright 2008-2016 Freescale Semiconductor, Inc.
 *  Copyright (c) 2008 by Embedded Alley Solution Inc.
 *
 *   Author: Pantelis Antoniou <pantelis@embeddedalley.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#define _GNU_SOURCE
#include <common.h>
#include <nand.h>
#include <linux/kernel.h>

#include "mtd.h"
#include "rand.h"

#define RAND_16K (16 * 1024)

const struct mtd_config default_mtd_config = {
	.search_exponent = 2,
	.data_setup_time = 80,
	.data_hold_time = 60,
	.address_setup_time = 25,
	.data_sample_time = 6,
	.row_address_size = 3,
	.column_address_size = 2,
	.read_command_code1 = 0x00,
	.read_command_code2 = 0x30,
	.boot_stream_major_version = 1,
	.boot_stream_minor_version = 0,
	.boot_stream_sub_version = 0,
	.ncb_version = 3,
};

/*
 * copy_bits - copy bits from one memory region to another
 * @dst: destination buffer
 * @dst_bit_off: bit offset we're starting to write at
 * @src: source buffer
 * @src_bit_off: bit offset we're starting to read from
 * @nbits: number of bits to copy
 *
 * This functions copies bits from one memory region to another, and is used by
 * the GPMI driver to copy ECC sections which are not guaranteed to be byte
 * aligned.
 *
 * src and dst should not overlap.
 *
 */
void copy_bits(uint8_t *dst, size_t dst_bit_off,
		    uint8_t *src, size_t src_bit_off,
		    size_t nbits)
{
	size_t i;
	size_t nbytes;
	uint32_t src_buffer = 0;
	size_t bits_in_src_buffer = 0;

	if (!nbits)
		return;

	/*
	 * Move src and dst pointers to the closest byte pointer and store bit
	 * offsets within a byte.
	 */
	src += src_bit_off / 8;
	src_bit_off %= 8;

	dst += dst_bit_off / 8;
	dst_bit_off %= 8;

	/*
	 * Initialize the src_buffer value with bits available in the first
	 * byte of data so that we end up with a byte aligned src pointer.
	 */
	if (src_bit_off) {
		src_buffer = src[0] >> src_bit_off;
		if (nbits >= (8 - src_bit_off)) {
			bits_in_src_buffer += 8 - src_bit_off;
		} else {
			src_buffer &= GENMASK(nbits - 1, 0);
			bits_in_src_buffer += nbits;
		}
		nbits -= bits_in_src_buffer;
		src++;
	}

	/* Calculate the number of bytes that can be copied from src to dst. */
	nbytes = nbits / 8;

	/* Try to align dst to a byte boundary. */
	if (dst_bit_off) {
		if (bits_in_src_buffer < (8 - dst_bit_off) && nbytes) {
			src_buffer |= src[0] << bits_in_src_buffer;
			bits_in_src_buffer += 8;
			src++;
			nbytes--;
		}

		if (bits_in_src_buffer >= (8 - dst_bit_off)) {
			dst[0] &= GENMASK(dst_bit_off - 1, 0);
			dst[0] |= src_buffer << dst_bit_off;
			src_buffer >>= (8 - dst_bit_off);
			bits_in_src_buffer -= (8 - dst_bit_off);
			dst_bit_off = 0;
			dst++;
			if (bits_in_src_buffer > 7) {
				bits_in_src_buffer -= 8;
				dst[0] = src_buffer;
				dst++;
				src_buffer >>= 8;
			}
		}
	}

	if (!bits_in_src_buffer && !dst_bit_off) {
		/*
		 * Both src and dst pointers are byte aligned, thus we can
		 * just use the optimized memcpy function.
		 */
		if (nbytes)
			memcpy(dst, src, nbytes);
	} else {
		/*
		 * src buffer is not byte aligned, hence we have to copy each
		 * src byte to the src_buffer variable before extracting a byte
		 * to store in dst.
		 */
		for (i = 0; i < nbytes; i++) {
			src_buffer |= src[i] << bits_in_src_buffer;
			dst[i] = src_buffer;
			src_buffer >>= 8;
		}
	}
	/* Update dst and src pointers */
	dst += nbytes;
	src += nbytes;

	/*
	 * nbits is the number of remaining bits. It should not exceed 8 as
	 * we've already copied as much bytes as possible.
	 */
	nbits %= 8;

	/*
	 * If there's no more bits to copy to the destination and src buffer
	 * was already byte aligned, then we're done.
	 */
	if (!nbits && !bits_in_src_buffer)
		return;

	/* Copy the remaining bits to src_buffer */
	if (nbits)
		src_buffer |= (*src & GENMASK(nbits - 1, 0)) <<
			      bits_in_src_buffer;
	bits_in_src_buffer += nbits;

	/*
	 * In case there were not enough bits to get a byte aligned dst buffer
	 * prepare the src_buffer variable to match the dst organization (shift
	 * src_buffer by dst_bit_off and retrieve the least significant bits
	 * from dst).
	 */
	if (dst_bit_off)
		src_buffer = (src_buffer << dst_bit_off) |
			     (*dst & GENMASK(dst_bit_off - 1, 0));
	bits_in_src_buffer += dst_bit_off;

	/*
	 * Keep most significant bits from dst if we end up with an unaligned
	 * number of bits.
	 */
	nbytes = bits_in_src_buffer / 8;
	if (bits_in_src_buffer % 8) {
		src_buffer |= (dst[nbytes] &
			       GENMASK(7, bits_in_src_buffer % 8)) <<
			      (nbytes * 8);
		nbytes++;
	}

	/* Copy the remaining bytes to dst */
	for (i = 0; i < nbytes; i++) {
		dst[i] = src_buffer;
		src_buffer >>= 8;
	}
}

/*
 * swap_block_mark - swap bbm
 * @data_off: pointer to the data address
 * @oob_off: pointer to the oob address
 * @nfc_geo: nfc_geometry structure
 * @wr_flag: 1 for write, 0 for read
 */
void swap_bad_block_mark(void *data, void *oob,
			struct nfc_geometry* nfc_geo, int wr_flag)
{
/*
 *         The situation is a little bit complicate since the it is not
 *         symmetric swap behavior.
 *
 *         Basic idea is swapping the data at block_mark_byte_offset,
 *         block_mark_bit_offset, denotes as dataX, with meta[0]. Since
 *         all related FCB data, including meta, FCB, parity check code,
 *         won't exceed NAND writesize, dataX is useless. But to protect
 *         the bad block mark, the correct behavior should be
 *
 *         +----------+------------------------+------------------------+
 *         |          |          dataX         |         meta[0]        |
 *         +----------+------------------------+------------------------+
 *         |   WRITE  |      swap to meta[0]   |     must set to 0xff   |
 *         +----------+------------------------+------------------------+
 *         |    READ  |          meta[0]       |     must be 0xff       |
 *         +----------+------------------------+------------------------+
 *
 *         the original value of dataX doesn't matter, the only thing need
 *         to save/restore is meta[0]
 */

	int byte_off = nfc_geo->block_mark_byte_offset;
	int bit_off = nfc_geo->block_mark_bit_offset;
	uint8_t *data_off = data;
	uint8_t *oob_off = oob;

	if (wr_flag) {
		data_off[byte_off] =
			(data_off[byte_off] & GENMASK(bit_off - 1, 0)) |
			(oob_off[0] << bit_off);
		data_off[byte_off + 1] =
			(data_off[byte_off + 1] << bit_off) |
			(oob_off[0] & GENMASK(7, bit_off - 1) >> (8- bit_off));
		oob_off[0] = 0xff;
	} else {
		oob_off[0] =
			((data_off[byte_off] & GENMASK(7, bit_off - 1)) >> bit_off) |
			((data_off[byte_off + 1] & GENMASK(bit_off - 1, 0)) << (8 - bit_off));
	}
}

int mtd_write_page(struct mtd_data *md, loff_t ofs, int ecc)
{
	int size;
	int r;
	void *data;
	void *oobdata;
	struct mtd_oob_ops ops;

	if ((!ecc) && md->raw_mode_flag) {
		int i;
		struct nfc_geometry *nfc_geo = &md->nfc_geometry;
		int eccbits = nfc_geo->ecc_strength * nfc_geo->gf_len;
		int chunksize = nfc_geo->ecc_chunkn_size_in_bytes;
		unsigned int src_bit_off;
		unsigned int oob_bit_off;
		int oob_bit_left;
		int ecc_chunk_count;

		data = malloc(mtd_writesize(md) + mtd_oobsize(md));
		if (!data) {
			printf("mtd: %s failed to allocate buffer\n", __func__);
			return -1;
		}
		oobdata = data + mtd_writesize(md);
		memset(data, 0, mtd_writesize(md) + mtd_oobsize(md));

		/* copy meta first */
		memcpy(oobdata, md->buf, nfc_geo->metadata_size_in_bytes);
		src_bit_off = nfc_geo->metadata_size_in_bytes * 8;
		oob_bit_off = src_bit_off;
		ecc_chunk_count = nfc_geo->ecc_chunk_count;

		/* if bch requires dedicate ecc for meta */
		if (nfc_geo->ecc_for_meta) {
			copy_bits(oobdata, oob_bit_off,
				       md->buf, src_bit_off,
				       eccbits);
			src_bit_off += eccbits;
			oob_bit_off += eccbits;
			ecc_chunk_count = nfc_geo->ecc_chunk_count - 1;
		}

		/* copy others */
		for (i = 0; i < ecc_chunk_count; i++) {
			copy_bits(data, i * chunksize * 8,
				       md->buf, src_bit_off,
				       chunksize * 8);
			src_bit_off += chunksize * 8;
			copy_bits(oobdata, oob_bit_off,
				       md->buf, src_bit_off,
				       eccbits);
			src_bit_off += eccbits;
			oob_bit_off += eccbits;
		}

		oob_bit_left = (mtd_writesize(md) + mtd_oobsize(md)) * 8 - src_bit_off;
		if (oob_bit_left) {
			copy_bits(oobdata, oob_bit_off,
				       md->buf, src_bit_off,
				       oob_bit_left);
		}

		/* all gpmi controller need to do bi swap, may use flag to do this later */
		swap_bad_block_mark(data, oobdata, nfc_geo, 1);
	} else {
		data = md->buf;
		oobdata = data + mtd_writesize(md);
	}

	/* make sure it's aligned to a page */
	if ((ofs % mtd_writesize(md)) != 0) {
		printf("mtd: %s failed\n", __func__);
		if ((!ecc) && md->raw_mode_flag)
			free(data);
		return -1;
	}

	/*
	 * for the legacy raw mode, since usefull data won't exceed writesize,
	 * set oobdata[0] to 0xff to protect bbm
	 */
	if ((!ecc) && (!md->raw_mode_flag))
		*(uint8_t *)oobdata = 0xff;

	size = mtd_writesize(md);

	ops.mode = ecc ? MTD_OPS_AUTO_OOB : MTD_OPS_RAW;
	ops.len = (size_t)size;
	ops.ooblen = (size_t)(ecc ? 0 : mtd_oobsize(md));
	ops.ooboffs = 0;
	ops.datbuf = (uint8_t *)data;
	ops.oobbuf = (uint8_t *)oobdata;

	r = mtd_write_oob(&md->mtd, ofs, &ops);
	if (!r)
		r = size;

	if ((!ecc) && md->raw_mode_flag)
		free(data);

	/* end of partition? */
	if (r == 0) {
		printf("mtd: %s written 0\n", __func__);
		return 0;
	}

	if (r != size) {
		printf("mtd: %s failed\n", __func__);
		return -1;
	}

	if (ecc)
		return size;

	return size + mtd_oobsize(md);
}

//------------------------------------------------------------------------------
// This function writes the search areas for a given BCB. It will write *two*
// search areas for a given BCB. If there are multiple chips, it will write one
// search area on each chip. If there is one chip, it will write two search
// areas on the first chip.
//
// md         A pointer to the current struct mtd_data.
// bcb_name   A pointer to a human-readable string that indicates what kind of
//            BCB we're writing. This string will only be used in log messages.
// ofs1       If there is one chips, the index of the
// ofs2
// ofs_mchip  If there are multiple chips, the index of the search area to write
//            on both chips.
// end        The number of consecutive search areas to be written.
// size       The size of the BCB data to be written.
// ecc        Indicates whether or not to use hardware ECC.
//------------------------------------------------------------------------------

int mtd_commit_bcb(struct mtd_data *md, char *bcb_name,
		   loff_t ofs1, loff_t ofs2, loff_t ofs_mchip,
		   loff_t end, size_t size, int ecc)
{
	loff_t end_index, search_area_indices[2], o;
	int err = 0, r;
	int i, j;
	unsigned stride_size_in_bytes;
	unsigned search_area_size_in_strides;
	unsigned search_area_size_in_bytes;
	unsigned count;

	vp(md, "-------------- Start to write the [ %s ] -----\n", bcb_name);
	//----------------------------------------------------------------------
	// Compute some important facts about geometry.
	//----------------------------------------------------------------------
	struct mtd_config *cfg = &md->cfg;

	stride_size_in_bytes        = cfg->stride_size_in_bytes;
	search_area_size_in_bytes   = cfg->search_area_size_in_bytes;
	search_area_size_in_strides = 1 << md->cfg.search_exponent;
	count = 2; //write two copy on mx6ull

	//----------------------------------------------------------------------
	// Loop over search areas for this BCB.
	//----------------------------------------------------------------------

	search_area_indices[0] = ofs1;
	search_area_indices[1] = ofs2;
	/* do not write the same position twice. */
	if (ofs1 == ofs2)
		count = 1;

	for (i = 0; !err && i < count; i++) {

		//--------------------------------------------------------------
		// Compute the search area index that marks the end of the
		// writing on this chip.
		//--------------------------------------------------------------

		end_index = search_area_indices[i] + end;

		//--------------------------------------------------------------
		// Loop over consecutive search areas to write.
		//--------------------------------------------------------------

		for (; search_area_indices[i] < end_index; search_area_indices[i]++) {

			//------------------------------------------------------
			// Compute the byte offset of the beginning of this
			// search area.
			//------------------------------------------------------
			o = search_area_indices[i] * search_area_size_in_bytes;

			//------------------------------------------------------
			// Loop over strides in this search area.
			//------------------------------------------------------

			for (j = 0; j < search_area_size_in_strides; j++, o += stride_size_in_bytes) {

				//----------------------------------------------
				// Write the page.
				//----------------------------------------------

				if (md->flags & F_VERBOSE)
					printf("mtd: Writing %s%d [ 0x%llx ] (%x) *\n", bcb_name, j, o, size);

				r = mtd_write_page(md, o, ecc);

				if (r != size) {
					printf("mtd: Failed to write %s: 0x%llx (%d)\n", bcb_name, o, r);
					err ++;
				}

			}

		}

	}

	if (md->flags & F_VERBOSE)
		printf("%s(%s): status %d\n\n", __func__, bcb_name, err);

	return err;
}

int write_boot_stream(struct mtd_data *md, ulong addr)
{
	int startpage, start, size;
	loff_t ofs, end;
	uchar *addr1;
	int i, r, chunk;
	struct fcb_block *fcb = &md->fcb.FCB_Block;

	vp(md, "---------- Start to write the [ %s ]----\n", (char*)md->private);
	for (i = 0; i < 2; i++) {
		if (i == 0) {
			startpage = fcb->m_u32Firmware1_startingPage;
			size      = fcb->m_u32PagesInFirmware1;
			end       = fcb->m_u32Firmware2_startingPage;
		} else {
			startpage = fcb->m_u32Firmware2_startingPage;
			size      = fcb->m_u32PagesInFirmware2;
			end       = mtd_size(md) / mtd_writesize(md);
		}

		start = startpage * mtd_writesize(md);
		size  = size      * mtd_writesize(md);
		end   = end       * mtd_writesize(md);

		if (md->flags & F_VERBOSE)
			vp(md, "mtd: Writting %s: 0x%08x - 0x%08x\n",
					(char*)md->private, start, start + size);

		/* Begin to write the image. */
		ofs = start;
		addr1 = (uchar *)addr;
		while (ofs < end && size > 0) {
			while (nand_block_isbad(&md->mtd, ofs) == 1) {
				vp(md, "mtd: Skipping bad block at 0x%llx\n", ofs);
				ofs += mtd_erasesize(md);
			}

			chunk = size;

			/*
			 * Check if we've entered a new block and, if so, erase
			 * it before beginning to write it.
			 */
			if (chunk > mtd_writesize(md))
				chunk = mtd_writesize(md);

			//fread(md->buf, 1, chunk, fp);
			if (memcpy(md->buf, (void *)addr1, chunk) == NULL) {
				printf("mtd: Failed (memcpy %d)\n", chunk);
				return -1;
			}

			addr1 = addr1 + chunk;

			/* write page */
			r = mtd_write_page(md, ofs, 1);
			if (r != mtd_writesize(md))
				printf("mtd: Failed to write BS @0x%llx (%d)\n", ofs, r);

			ofs += mtd_writesize(md);
			size -= chunk;
		}

		/*
		 * Write one safe guard page:
		 *  The Image_len of uboot is bigger then the real size of
		 *  uboot by 1K. The ROM will get all 0xff error in this case.
		 *  So we write one more page for safe guard.
		 */
		memset(md->buf, 0, mtd_writesize(md));
		r = mtd_write_page(md, ofs, 1);
		if (r != mtd_writesize(md))
			printf("Failed to write safe page\n");
		vp(md, "mtd: We write one page for save guard. *\n");

		if (ofs >= end) {
			printf("mtd: Failed to write\n");
			return -1;
		}
	}
	return 0;
}

int v6_rom_mtd_commit_structures(struct mtd_data *md, ulong addr)
{
	int size, i, r;
	loff_t ofs;
	char buf[2048 + 64] = {0};
	struct mtd_config *cfg = &md->cfg;

	/* [1] Write the FCB search area. */
	size = mtd_writesize(md) + mtd_oobsize(md);
	memset(md->buf, 0, size);
	r = fcb_encrypt(&md->fcb, buf, size, 3);
	if (r < 0)
		return r;

	memcpy(md->buf + 10, buf, 2048 + 64 -10);
	mtd_commit_bcb(md, "FCB", 0, 0, 0, 1, size, false);

	/* [2] Write the DBBT search area. */
	memset(md->buf, 0, mtd_writesize(md));
	memcpy(md->buf, &(md->dbbt50), sizeof(md->dbbt50));
	mtd_commit_bcb(md, "DBBT", 1, 1, 1, 1, mtd_writesize(md), true);

	/* Write the DBBT table area. */
	memset(md->buf, 0, mtd_writesize(md));
	if (md->dbbt50.DBBT_Block.v3.m_u32DBBTNumOfPages > 0 && md->bbtn[0] != NULL) {
		memcpy(md->buf, md->bbtn[0], sizeof(*md->bbtn[0]));

		ofs = cfg->search_area_size_in_bytes;

		for (i = 0; i < 4; i++, ofs += cfg->stride_size_in_bytes) {
			vp(md, "mtd: PUTTING down DBBT%d BBTN%d @0x%llx (0x%x)\n",
				i, 0, ofs + 4 * mtd_writesize(md), mtd_writesize(md));

			r = mtd_write_page(md, ofs + 4 * mtd_writesize(md), 1);
			if (r != mtd_writesize(md)) {
				printf("mtd: Failed to write BBTN @0x%llx (%d)\n", ofs, r);
			}
		}
	}

	/* [3] Write the two boot streams. */
	return write_boot_stream(md, addr);
}

static void rom_boot_setting(struct mtd_data *md)
{
	struct mtd_config *cfg = &md->cfg;

	cfg->stride_size_in_bytes = PAGES_PER_STRIDE * mtd_writesize(md);
	cfg->search_area_size_in_bytes =
		(1 << cfg->search_exponent) * cfg->stride_size_in_bytes;
	cfg->search_area_size_in_pages =
		(1 << cfg->search_exponent) * PAGES_PER_STRIDE;
}

/* calculate the geometry ourselves. */
static int cal_nfc_geometry(struct mtd_data *md)
{
	struct nfc_geometry *geo = &md->nfc_geometry;
	struct mtd_info *mtd = &md->mtd; /* first one */
	unsigned int block_mark_bit_offset;
	struct nand_chip *chip = mtd->priv;

	/* The two are fixed, please change them when the driver changes. */
	geo->gf_len = 13;
	geo->ecc_chunkn_size_in_bytes = geo->ecc_chunk0_size_in_bytes = 512;

	if (mtd->oobsize > geo->ecc_chunkn_size_in_bytes) {
		geo->gf_len = 14;
		geo->ecc_chunkn_size_in_bytes *= 2;
	}

	geo->ecc_strength = round_up(chip->ecc_strength_ds, 2);
	geo->metadata_size_in_bytes = 10;

	geo->ecc_chunk_count = mtd->writesize / geo->ecc_chunkn_size_in_bytes;
	geo->page_size_in_bytes = mtd->writesize + geo->metadata_size_in_bytes +
		(geo->gf_len * geo->ecc_strength * geo->ecc_chunk_count) / 8;

	/*
	 * We need to compute the byte and bit offsets of
	 * the physical block mark within the ECC-based view of the page.
	 *
	 * NAND chip with 2K page shows below:
	 *                                             (Block Mark)
	 *                                                   |      |
	 *                                                   |  D   |
	 *                                                   |<---->|
	 *                                                   V      V
	 *    +---+----------+-+----------+-+----------+-+----------+-+
	 *    | M |   data   |E|   data   |E|   data   |E|   data   |E|
	 *    +---+----------+-+----------+-+----------+-+----------+-+
	 *
	 * The position of block mark moves forward in the ECC-based view
	 * of page, and the delta is:
	 *
	 *                   E * G * (N - 1)
	 *             D = (---------------- + M)
	 *                          8
	 *
	 * With the formula to compute the ECC strength, and the condition
	 *       : C >= O         (C is the ecc chunk size)
	 *
	 * It's easy to deduce to the following result:
	 *
	 *         E * G       (O - M)      C - M
	 *      ----------- <= ------- <  ---------
	 *           8            N        (N - 1)
	 *
	 *  So, we get:
	 *
	 *                   E * G * (N - 1)
	 *             D = (---------------- + M) < C
	 *                          8
	 *
	 *  The above inequality means the position of block mark
	 *  within the ECC-based view of the page is still in the data chunk,
	 *  and it's NOT in the ECC bits of the chunk.
	 *
	 *  Use the following to compute the bit position of the
	 *  physical block mark within the ECC-based view of the page:
	 *          (page_size - D) * 8
	 */
	block_mark_bit_offset = mtd->writesize * 8 -
		(geo->ecc_strength * geo->gf_len * (geo->ecc_chunk_count - 1)
				+ geo->metadata_size_in_bytes * 8);

	geo->block_mark_byte_offset = block_mark_bit_offset / 8;
	geo->block_mark_bit_offset  = block_mark_bit_offset % 8;

	return 0;
}

void mtd_close(struct mtd_data *md)
{
	if (md == NULL)
		return;

	if (md->buf)
		free(md->buf);

	if (md->bbtn[0])
		free(md->bbtn[0]);
	if (md->bbtn[1])
		free(md->bbtn[1]);

	free(md);
}

struct mtd_data *mtd_open(const struct mtd_config *cfg, int flags)
{
	struct mtd_data *md;
	struct mtd_info *mtd;
	struct nfc_geometry *geo;

	md = malloc(sizeof(*md));
	if (md == NULL)
		goto out;
	memset(md, 0, sizeof(*md));
	md->flags = flags;

	if (cfg == NULL)
		cfg = &default_mtd_config;

	md->cfg = *cfg;

	md->raw_mode_flag = 1;

	/* get info about the mtd device (partition) */
	mtd = get_mtd_device(NULL, 0);
	if (IS_ERR(mtd)) {
		printf("get_mtd_device error..\n");
		goto out;
	}		

	md->mtd = *mtd;

	md->mtd.size = 4194304;
	/* verify it's a nand */
	if (mtd->type != MTD_NANDFLASH
		&& mtd->type != MTD_MLCNANDFLASH) {
		printf("mtd: The device is not NAND\n");
		goto out;
	}

	/* verify it's a supported geometry */
	if (mtd->writesize + mtd->oobsize != 2048 + 64 &&
		mtd->writesize + mtd->oobsize != 4096 + 128 &&
		mtd->writesize + mtd->oobsize != 4096 + 224 &&
		mtd->writesize + mtd->oobsize != 4096 + 218 &&
		mtd->writesize + mtd->oobsize != 8192 + 376 &&
		mtd->writesize + mtd->oobsize != 8192 + 512) {
		printf("mtd: The device is unsupported geometry (%d/%d)\n", mtd->writesize, mtd->oobsize);
		goto out;
	}

	/* Set up booting parameters */
	rom_boot_setting(md);

	md->buf = malloc(mtd_writesize(md) + mtd_oobsize(md));
	if (md->buf == NULL) {
		printf("mtd: unable to allocate page buffer\n");
		goto out;
	}

	/* Parse the NFC geometry. */
	if (cal_nfc_geometry(md)) {
		printf("mtd: unable to parse NFC geometry\n");
		goto out;
	}
	geo = &md->nfc_geometry;
	vp(md, "NFC geometry :\n");
	vp(md, "\tECC Strength       : %d\n", geo->ecc_strength);
	vp(md, "\tPage Size in Bytes : %d\n", geo->page_size_in_bytes);
	vp(md, "\tMetadata size      : %d\n", geo->metadata_size_in_bytes);
	vp(md, "\tECC Chunk Size in byte : %d\n", geo->ecc_chunkn_size_in_bytes);
	vp(md, "\tECC Chunk count        : %d\n", geo->ecc_chunk_count);
	vp(md, "\tBlock Mark Byte Offset : %d\n", geo->block_mark_byte_offset);
	vp(md, "\tBlock Mark Bit Offset  : %d\n", geo->block_mark_bit_offset);
	vp(md, "====================================================\n");

	return md;
out:
	mtd_close(md);
	return NULL;
}

static int fill_fcb(struct mtd_data *md, int len)
{
	BCB_ROM_BootBlockStruct_t *fcb = &md->fcb;
	struct mtd_config *cfg   = &md->cfg;
	struct fcb_block *b      = &fcb->FCB_Block;
	FCB_ROM_NAND_Timing_t *t = &b->m_NANDTiming;
	struct nfc_geometry *geo = &md->nfc_geometry;
	unsigned int  max_boot_stream_size_in_bytes;
	unsigned int  boot_stream_size_in_bytes;
	unsigned int  boot_stream_size_in_pages;
	unsigned int  boot_stream1_pos;
	unsigned int  boot_stream2_pos;

	if ((cfg->search_area_size_in_bytes * 2) > mtd_size(md)) {
		printf("mtd: mtd size too small\n");
		return -1;
	}

	/*
	 * Figure out how large a boot stream the target MTD could possibly
	 * hold.
	 *
	 * The boot area will contain both search areas and two copies of the
	 * boot stream.
	 */
	max_boot_stream_size_in_bytes =
		(mtd_size(md) - cfg->search_area_size_in_bytes * 2) / 2;

	/* Figure out how large the boot stream is. */

	boot_stream_size_in_bytes = len;

	boot_stream_size_in_pages =
		(boot_stream_size_in_bytes + (mtd_writesize(md) - 1)) /
					mtd_writesize(md);
	/* Check if the boot stream will fit. */
	if (boot_stream_size_in_bytes >= max_boot_stream_size_in_bytes) {
		printf("mtd: bootstream too large\n");
		return -1;
	}

	/* Compute the positions of the boot stream copies. */
	boot_stream1_pos = 2 * cfg->search_area_size_in_bytes;
	boot_stream2_pos = boot_stream1_pos + max_boot_stream_size_in_bytes;

	vp(md, "mtd: max_boot_stream_size_in_bytes = %d\n"
		"mtd: boot_stream_size_in_bytes = %d\n"
		"mtd: boot_stream_size_in_pages = %d\n",
			max_boot_stream_size_in_bytes,
			boot_stream_size_in_bytes,
			boot_stream_size_in_pages);
	vp(md, "mtd: #1 0x%08x - 0x%08x (0x%08x)\n"
		"mtd: #2 0x%08x - 0x%08x (0x%08x)\n",
			boot_stream1_pos,
			boot_stream1_pos + max_boot_stream_size_in_bytes,
			boot_stream1_pos + boot_stream_size_in_bytes,
			boot_stream2_pos,
			boot_stream2_pos + max_boot_stream_size_in_bytes,
			boot_stream2_pos + boot_stream_size_in_bytes);

	memset(fcb, 0, sizeof(*fcb));

	fcb->m_u32FingerPrint	= FCB_FINGERPRINT;
	fcb->m_u32Version	= FCB_VERSION_1;

	/* timing */
	t->m_u8DataSetup    = cfg->data_setup_time;
	t->m_u8DataHold     = cfg->data_hold_time;
	t->m_u8AddressSetup = cfg->address_setup_time;
	t->m_u8DSAMPLE_TIME = cfg->data_sample_time;

	/* fcb block */
	b->m_u32PageDataSize	= mtd_writesize(md);
	b->m_u32TotalPageSize	= mtd_writesize(md) + mtd_oobsize(md);
	b->m_u32SectorsPerBlock	= mtd_erasesize(md) / mtd_writesize(md);

	b->m_u32EccBlockNEccType = b->m_u32EccBlock0EccType =
					geo->ecc_strength >> 1;
	if (geo->ecc_for_meta)
		b->m_u32EccBlock0Size	= 0;
	else
		b->m_u32EccBlock0Size	= geo->ecc_chunk0_size_in_bytes;
	b->m_u32EccBlockNSize	= geo->ecc_chunkn_size_in_bytes;
	b->m_u32MetadataBytes	= geo->metadata_size_in_bytes;
	b->m_u32NumEccBlocksPerPage = geo->ecc_chunk_count - 1;

	b->m_u32Firmware1_startingPage = boot_stream1_pos / mtd_writesize(md);
	b->m_u32Firmware2_startingPage = boot_stream2_pos / mtd_writesize(md);
	b->m_u32PagesInFirmware1       = boot_stream_size_in_pages;
	b->m_u32PagesInFirmware2       = boot_stream_size_in_pages;

	b->m_u32DBBTSearchAreaStartAddress = cfg->search_area_size_in_pages;
	b->m_u32BadBlockMarkerByte     = geo->block_mark_byte_offset;
	b->m_u32BadBlockMarkerStartBit = geo->block_mark_bit_offset;
	b->m_u32BBMarkerPhysicalOffset = mtd_writesize(md);
	b->m_u32BCHType = geo->gf_len == 14 ? 1 : 0;

	return 0;
}


/* fill in Discoverd Bad Block Table. */
static int fill_dbbt(struct mtd_data *md)
{
	BCB_ROM_BootBlockStruct_t *dbbt;

	dbbt = &md->dbbt50;
	memset(dbbt, 0, sizeof(*dbbt));

	dbbt->m_u32FingerPrint = DBBT_FINGERPRINT2;
	dbbt->m_u32Version = DBBT_VERSION_1;

	return 0;
}

int v4_rom_mtd_init(struct mtd_data *md, int len)
{
	int ret;

	ret = fill_fcb(md, len);
	if (ret)
		return ret;
	return fill_dbbt(md);
}

