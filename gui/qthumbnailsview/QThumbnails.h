/*
 * QThumbnails.h
 *
 *  Created on: Mar 6, 2020
 *      Author: amyznikov
 */

#ifndef __QThumbnails_h__
#define __QThumbnails_h__

#include <QtGui/QtGui>

QSize compute_thumbnail_size(QSize srcSize, int max_thumb_size);
QStringList getSupportedThumbnailsExtensions();
QImage  loadThumbnailImage(const QString & pathFileName, int thumb_size);
QIcon   loadThumbnailIcon(const QString & pathFileName, int maxSize);
QPixmap loadThumbnailPixmap(const QString & pathFileName, int maxSize);
const char ** thumbnail_textfile_suffixes();
const char ** thumbnail_plyfile_suffixes();

bool isTextFileSuffix(const QString & suffix);
bool isPlyFileSuffix(const QString & suffix);
bool isTextFile(const QString & abspath);
bool isPlyFile(const QString & abspath);

#endif /* __QThumbnails_h__ */
