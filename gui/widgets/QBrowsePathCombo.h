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

  void setFileFilter(const QString & v);
  const QString & fileFilter() const;

  void addPath(const QString & path);
  void setCurrentPath(const QString & path) ;
  QString currentPath(void) const;

  bool hasChanges(void) const;
  void setHasChanges(bool f);

Q_SIGNALS:
  void pathChanged();

private Q_SLOTS:
  void onBrowseForPath(void);

protected:
  bool eventFilter(QObject *watched, QEvent *event) override;

protected:
  QLabel * label = nullptr;
  QComboBox * combo = nullptr;
  QToolButton * button = nullptr;
  QString fileDialogCaption;
  QString _labelText;
  QString _fileFilter;
  QFileDialog::FileMode _fileMode = QFileDialog::AnyFile;
  QFileDialog::AcceptMode _acceptMode = QFileDialog::AcceptOpen;
  QFileDialog::ViewMode _viewMode = QFileDialog::ViewMode::Detail;
  bool _showDirsOnly = false;
  bool _hasChanges = false;

private:
  void construct(void);
};


///////////////////////////////////////////////////////////////////////////////
#endif /* __QBrowsePathCombo_h__ */
