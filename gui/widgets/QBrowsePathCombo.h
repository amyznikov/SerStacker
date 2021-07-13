/*
 * QBrowsePathCombo.h
 *
 *  Created on: Jan 7, 2017
 *      Author: amyznikov
 */

#pragma once

#ifndef __QBrowsePathCombo_h__
#define __QBrowsePathCombo_h__

#include <QtWidgets/QtWidgets>

///////////////////////////////////////////////////////////////////////////////


class QBrowsePathCombo
    : public QWidget
{
  Q_OBJECT;
public:

  typedef QBrowsePathCombo ThisClass;
  typedef QWidget Base;


  QBrowsePathCombo(QWidget *parent = Q_NULLPTR);
  QBrowsePathCombo(const QString & label, QFileDialog::FileMode mode = QFileDialog::AnyFile, QWidget *parent = Q_NULLPTR);

  void setLabel(const QString & label);
  void setFileDialogCaption(const QString & caption);
  void setFileDialogMode(QFileDialog::FileMode mode);

  void addPath(const QString & path, bool emitHasChages = false);

  void setCurrentPath(const QString & path, bool emitHasChages = false) ;
  QString currentPath(void) const;

  bool hasChanges(void) const;
  void setHasChanges(bool f);

signals:
  void pathSelected(const QString & path); // selected by 'browse button using QFileDialog
  void pathChanged(); // any change


private slots:
  void onBrowseForPath(void);
  void currentTextChanged(const QString &);

private:
  QLabel * label = Q_NULLPTR;
  QComboBox * combo = Q_NULLPTR;
  QToolButton * button = Q_NULLPTR;
  QString fileDialogCaption;
  QString labelText_;
  QFileDialog::FileMode fileDialogMode = QFileDialog::AnyFile;
  QFileDialog::ViewMode fileDialogViewMode = QFileDialog::ViewMode::List;
  bool hasChanges_ = false;
  bool enableEmitChagesEvent_ = true;

private:
  void construct(void);
};


///////////////////////////////////////////////////////////////////////////////
#endif /* __QBrowsePathCombo_h__ */
