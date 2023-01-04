/*
 * style.h
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef _gui_style_h__
#define _gui_style_h__

#include <QtGui/QtGui>



void setIconStyleSelector(const QString & selector);
const QString & iconStyleSelector();

QIcon getIcon(const QString & name);
QPixmap getPixmap(const QString & name);



#endif /* _gui_style_h__ */
