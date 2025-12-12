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

  explicit QTextFileViewer(QWidget * parent = nullptr);

  QToolBar * toolbar();

  void showTextFile(const std::string & source);
  void showTextFile(const QString & source);
  void clear();


  QString currentFileName() const;

  bool findString(const QString & text, bool caseSensitive, bool wholeWords, bool backward);
  bool findNext();
  bool findPrevious();
  void copySelectionToClipboard();


Q_SIGNALS:
  void visibilityChanged(bool visible);
  void currentFileNameChanged();

protected:
  void showEvent(QShowEvent *) override;
  void hideEvent(QHideEvent *) override;

protected:
  QString _currentFileName;
  QVBoxLayout * _layout = nullptr;
  QToolBar * _toolbar = nullptr;
  QPlainTextEdit * _textBrowser = nullptr;

  QString _searchText;
  QTextDocument::FindFlags _searchFlags;
};

class QFindTextDialog:
    public QDialog
{
  Q_OBJECT
public:
  typedef QFindTextDialog ThisClass;
  typedef QDialog Base;

  explicit QFindTextDialog(QWidget *parent = nullptr);

  QString getSearchString() const;
  bool isCaseSensitive() const;
  bool isWholeWords() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);
  void findTextRequested(const QString & text, bool caseSensitive, bool wholeWords, bool backward);

protected:
  void showEvent(QShowEvent *) final;
  void hideEvent(QHideEvent *) final;

private:
  QLineEdit *searchLine_ctl;
  QCheckBox *caseSensitiveCheckBox_ctl;
  QCheckBox *wholeWordsCheckBox_ctl;
  QCheckBox *searchBackward_ctl;
  QPushButton *findButton_ctl;
  QPushButton *cancelButton_ctl;
};

#endif /* __QTextFileViewer_h__ */
