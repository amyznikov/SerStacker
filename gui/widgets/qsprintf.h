/*
 * qsprintf.h
 *
 *  Created on: Jan 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __qsprintf_h__
#define __qsprintf_h__

#include <QtCore/QtCore>


QString qsprintf(const char * format, ...)
  Q_ATTRIBUTE_FORMAT_PRINTF(1, 0);

inline QString qsprintf(const char * format, ...)
{
  va_list arglist;
  va_start(arglist, format);

#if QT_VERSION < QT_VERSION_CHECK(5, 14, 0)
  QString msg;
  msg.vsprintf(format, arglist);
#else
  QString msg = QString::vasprintf(format, arglist);
#endif

  va_end(arglist);

  return msg;
}


#endif /* __qsprintf_h__ */
