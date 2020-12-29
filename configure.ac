#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.60)
AC_INIT(dvminigrab, 1.0, jonathan@impactstudiopro.com)
AC_CONFIG_SRCDIR([dvminigrab.c])
dnl AC_CONFIG_HEADER([config.h])

# Checks for programs.
AC_PROG_CC

# Checks for libraries.
# FIXME: Replace `main' with a function in `-lavc1394':
AC_CHECK_LIB([avc1394], [avc1394_init_target])
# FIXME: Replace `main' with a function in `-lclocksync':
AC_CHECK_LIB([clocksync], [clocksync_open])
# FIXME: Replace `main' with a function in `-liec61883':
AC_CHECK_LIB([iec61883], [iec61883_amdtp_xmit_stop])
# FIXME: Replace `main' with a function in `-lpthread':
AC_CHECK_LIB([pthread], [pthread_mutex_init])
# FIXME: Replace `main' with a function in `-lraw1394':
AC_CHECK_LIB([raw1394], [raw1394_iso_xmit_init])
# FIXME: Replace `main' with a function in `-lrom1394':
AC_CHECK_LIB([rom1394], [rom1394_get_size])

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([fcntl.h stdint.h stdlib.h string.h sys/mount.h sys/param.h sys/statfs.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_C_CONST
AC_C_INLINE
AC_TYPE_INT16_T
AC_TYPE_INT64_T
AC_TYPE_SIZE_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT64_T
AC_C_VOLATILE

# Checks for library functions.
AC_FUNC_MALLOC
AC_FUNC_MMAP
AC_TYPE_SIGNAL
AC_CHECK_FUNCS([alarm ftruncate getcwd gethostname memset munmap strchr strerror strtoull])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
