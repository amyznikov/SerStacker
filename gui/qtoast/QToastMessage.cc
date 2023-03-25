/*
 * QToastMessage.cc
 *
 *  Created on: Mar 24, 2023
 *      Author: amyznikov
 */

#include "QToastMessage.h"

QToastMessage::QToastMessage()
{
}

QToastMessage * QToastMessage::singleton(QWidget * parent)
{
  return nullptr;
}


void QToastMessage::information(QWidget * parent, const QString & msg)
{

}

void QToastMessage::warning(QWidget * parent, const QString & msg)
{

}

void QToastMessage::critical(QWidget * parent, const QString & msg)
{

}
