# This is an example file to generate boot.scr - a boot script for U-Boot
# Generate boot.scr:
# ./tools/mkimage -c none -A arm -T script -d autoboot.cmd boot.scr
#
# It requires a list of environment variables to be defined before load:
# platform dependent: boardname, fdt_file, console
# system dependent: mmcdev, mmcpart, mmcpart, rootfstype
#
setenv fdtaddr     "0x83000000"
setenv loaddtb     "load mmc ${mmcdev}:${mmcpart} ${fdtaddr} ${fdt_file}"
setenv loadkernel  "load mmc ${mmcdev}:${mmcpart} '${kerneladdr}' '${kernelname}'"
setenv kernel_args "setenv bootargs console=${console},${baudrate} cma=96M root=${mmcroot}"

#### Routine: check_dtb - check that target.dtb exists on boot partition
setenv check_dtb "
if test -e mmc '${mmcdev}':'${mmcpart}' '${fdt_file}'; then
	run loaddtb;
	setenv fdt_addr ${fdtaddr};
else
	echo Warning! Booting without DTB: '${fdt_file}'!;
	setenv fdt_addr;
fi;"

#### Routine: setboot_zimg - prepare env to boot zImage
setenv setboot_zimg "
	setenv kerneladdr 0x80800000;
	setenv kernelname zImage;
	run check_dtb;"

#### Routine: boot_img - boot the kernel after env setup
setenv boot_img "
	run loadkernel;
	run kernel_args;
	bootz '${kerneladdr}' '-' '${fdt_addr}';"

#### Routine: autoboot - choose proper boot path
setenv autoboot "
if test -e mmc 0:${mmcpart} zImage; then
	echo Found kernel image: zImage;
	run setboot_zimg;
	run boot_img;
fi;"

#### Execute the defined autoboot macro
run autoboot
