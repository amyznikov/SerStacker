/*
 * version.h
 *
 *  Created on September 5, 2019
 *      Author: amyznikov
 */
#pragma once
#ifndef __qwcc_version_h__
#define __qwcc_version_h__

#define QWCC_VERSION_MAJOR    0
#define QWCC_VERSION_MINOR    0
#define QWCC_VERSION_REVISION 1
#define QWCC_VERSION_STATUS   "-pre"

#define QWCCAUX_STR_EXP(__A)  #__A
#define QWCCAUX_STR(__A)      QWCCAUX_STR_EXP(__A)

#define QWCCAUX_STRW_EXP(__A)  L ## #__A
#define QWCCAUX_STRW(__A)      QWCCAUX_STRW_EXP(__A)

#define QWCC_VERSION   \
  QWCCAUX_STR(QWCC_VERSION_MAJOR) "." QWCCAUX_STR(QWCC_VERSION_MINOR) "." QWCCAUX_STR(QWCC_VERSION_REVISION) QWCC_VERSION_STATUS




#endif /* __qwcc_version_h__ */
