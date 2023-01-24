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

class QBrowsePathCombo :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QBrowsePathCombo ThisClass;
  typedef QWidget Base;

  QBrowsePathCombo(QWidget *parent = nullptr);
  QBrowsePathCombo(const QString & label, QFileDialog::AcceptMode acceptMode,
      QFileDialog::FileMode mode = QFileDialog::AnyFile,  QWidget *parent = nullptr);

  void setShowDirsOnly(bool v);
  bool showDirsOnly() const;

  void setLabel(const QString & label);
  void setFileDialogCaption(const QString & caption);

  void setFileMode(QFileDialog::FileMode mode);
  QFileDialog::FileMode fileMode() const;

  void setAcceptMode(QFileDialog::AcceptMode mode);
  QFileDialog::AcceptMode acceptMode() const;

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
  QLabel * label = nullptr;
  QComboBox * combo = nullptr;
  QToolButton * button = nullptr;
  QString fileDialogCaption;
  QString labelText_;
  QFileDialog::FileMode fileMode_ = QFileDialog::AnyFile;
  QFileDialog::AcceptMode acceptMode_ = QFileDialog::AcceptOpen;
  QFileDialog::ViewMode viewMode_ = QFileDialog::ViewMode::Detail;
  bool showDirsOnly_ = false;
  bool hasChanges_ = false;
  bool enableEmitChagesEvent_ = true;


private:
  void construct(void);
};


///////////////////////////////////////////////////////////////////////////////
#endif /* __QBrowsePathCombo_h__ */
