/*
 * QThumbnails.h
 *
 *  Created on: Mar 6, 2020
 *      Author: amyznikov
 */

#ifndef __QThumbnails_h__
#define __QThumbnails_h__

#include <QtGui/QtGui>

QStringList getSupportedThumbnailsExtensions();
QImage  loadThumbnailImage(const QString & pathFileName, int thumb_size);
QIcon   loadThumbnailIcon(const QString & pathFileName, int maxSize);
QPixmap loadThumbnailPixmap(const QString & pathFileName, int maxSize);


#endif /* __QThumbnails_h__ */
