/*
 * QTextFileViewer.h
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QTextFileViewer_h__
#define __QTextFileViewer_h__

#include <QtWidgets/QtWidgets>

class QTextFileViewer
    : public QWidget
{
  Q_OBJECT;
public:

  typedef QTextFileViewer ThisClass;
  typedef QWidget Base;

  QTextFileViewer(QWidget * parent = Q_NULLPTR);

  QToolBar * toolbar() const;

  void showTextFile(const std::string & source);
  void showTextFile(const QString & source);
  void clear();


  QString currentFileName() const;

signals:
  void currentFileNameChanged();

protected:
  QString currentFileName_;
  QVBoxLayout * layout_ = Q_NULLPTR;
  QToolBar * toolbar_ = Q_NULLPTR;
  //QTextEdit * textBrowser_ = Q_NULLPTR;
  QPlainTextEdit * textBrowser_ = Q_NULLPTR;
};

#endif /* __QTextFileViewer_h__ */
