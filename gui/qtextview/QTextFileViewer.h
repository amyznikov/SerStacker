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

class QTextFileViewer :
    public QWidget
{
  Q_OBJECT;
public:

  typedef QTextFileViewer ThisClass;
  typedef QWidget Base;

  QTextFileViewer(QWidget * parent = nullptr);

  QToolBar * toolbar();

  void showTextFile(const std::string & source);
  void showTextFile(const QString & source);
  void clear();


  QString currentFileName() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);
  void currentFileNameChanged();

protected:
  void showEvent(QShowEvent *) override;
  void hideEvent(QHideEvent *) override;

protected:
  QString currentFileName_;
  QVBoxLayout * layout_ = nullptr;
  QToolBar * toolbar_ = nullptr;
  //QTextEdit * textBrowser_ = nullptr;
  QPlainTextEdit * textBrowser_ = nullptr;
};

#endif /* __QTextFileViewer_h__ */
