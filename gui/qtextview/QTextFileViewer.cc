/*
 * QTextFileViewer.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#include "QTextFileViewer.h"
#include <core/debug.h>

QTextFileViewer::QTextFileViewer(QWidget * parent) :
  Base(parent)
{
  _layout = new QVBoxLayout(this);
  _layout->setContentsMargins(0, 0, 0, 0);

  _textBrowser = new QPlainTextEdit(this);
  _textBrowser->setReadOnly(true);
  _textBrowser->setLineWrapMode(QPlainTextEdit::NoWrap);
  _textBrowser->setWordWrapMode(QTextOption::WrapMode::NoWrap);
  _textBrowser->setFont(QFont("monospace"));

  // Set selection colors using a stylesheet
  _textBrowser->setStyleSheet(
       "QPlainTextEdit {"
         "selection-background-color: #0078d7;" // Background color of selected text
         "selection-color: #ffffff;"           // Text color of selected text
       "}"
   );

   _layout->addWidget(_textBrowser, 100);

}

void QTextFileViewer::showEvent(QShowEvent * e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QTextFileViewer::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

QToolBar * QTextFileViewer::toolbar()
{
  if ( !_toolbar ) {
    _toolbar = new QToolBar(this);
    _toolbar->setToolButtonStyle(Qt::ToolButtonIconOnly);
    _toolbar->setOrientation(Qt::Horizontal);
    _toolbar->setIconSize(QSize(16,16));
    _layout->insertWidget(0, _toolbar, 1);
  }
  return _toolbar;
}

QString QTextFileViewer::currentFileName() const
{
  return _currentFileName;
}

void QTextFileViewer::clear()
{
  _textBrowser->clear();
  _currentFileName.clear();
  Q_EMIT currentFileNameChanged();
}

void QTextFileViewer::showTextFile(const std::string & pathfilename)
{
  showTextFile(QString::fromStdString(pathfilename));
}

void QTextFileViewer::showTextFile(const QString & pathfilename)
{
  _textBrowser->clear();

  if ( !(_currentFileName = pathfilename).isEmpty() ) {

    // QWaitCursor wait(this);

    QFile file(_currentFileName);

    if ( file.open(QIODevice::ReadOnly | QIODevice::Text) ) {
      _textBrowser->setPlainText(file.readAll());
    }

  }

  Q_EMIT currentFileNameChanged();
}

bool QTextFileViewer::findString(const QString &text, bool caseSensitive, bool wholeWords, bool backward)
{
  _searchText = text;
  _searchFlags = QTextDocument::FindFlags();

  if (caseSensitive) {
    _searchFlags |= QTextDocument::FindCaseSensitively;
  }
  if (wholeWords) {
    _searchFlags |= QTextDocument::FindWholeWords;
  }

  if (backward) {
    _searchFlags |= QTextDocument::FindBackward;
  }

  return _textBrowser->find(text, _searchFlags);
}

bool QTextFileViewer::findNext()
{
  if ( _searchText.isEmpty() ) {
    return false;
  }

  const QTextDocument::FindFlags flags =
      _searchFlags & ~QTextDocument::FindBackward;

  return _textBrowser->find(_searchText, flags);
}

bool QTextFileViewer::findPrevious()
{
  if ( _searchText.isEmpty() ) {
    return false;
  }

  const QTextDocument::FindFlags flags =
      _searchFlags | QTextDocument::FindBackward;

  return _textBrowser->find(_searchText, flags);
}

void QTextFileViewer::copySelectionToClipboard()
{
  _textBrowser->copy();
}


QFindTextDialog::QFindTextDialog(QWidget *parent) :
    QDialog(parent)
{
  setWindowTitle("Find Text");

  searchLine_ctl = new QLineEdit(this);
  searchLine_ctl->setPlaceholderText("Enter text to find...");

  caseSensitiveCheckBox_ctl = new QCheckBox("Case sensitive", this);
  wholeWordsCheckBox_ctl = new QCheckBox("Whole words", this);
  searchBackward_ctl = new QCheckBox("Search Backward", this);

  findButton_ctl = new QPushButton("Find", this);
  cancelButton_ctl = new QPushButton("Cancel", this);


  QHBoxLayout *buttonLayout = new QHBoxLayout;
  buttonLayout->addWidget(findButton_ctl);
  buttonLayout->addWidget(cancelButton_ctl);

  QVBoxLayout *mainLayout = new QVBoxLayout(this);
  mainLayout->addWidget(searchLine_ctl);
  mainLayout->addWidget(caseSensitiveCheckBox_ctl);
  mainLayout->addWidget(wholeWordsCheckBox_ctl);
  mainLayout->addWidget(searchBackward_ctl);
  mainLayout->addLayout(buttonLayout);

  connect(cancelButton_ctl, &QPushButton::clicked, this, &QFindTextDialog::reject);

  connect(findButton_ctl, &QPushButton::clicked,
      [this]() {

        QString text = searchLine_ctl->text();
        if ( !text.isEmpty( ) ) {
          const bool caseSensitive = caseSensitiveCheckBox_ctl->isChecked();
          const bool wholeWords = wholeWordsCheckBox_ctl->isChecked();
          const bool backward = searchBackward_ctl->isChecked();
          Q_EMIT findTextRequested(text, caseSensitive, wholeWords, backward);
        }
      });
}

QString QFindTextDialog::getSearchString() const
{
  return searchLine_ctl->text();
}

bool QFindTextDialog::isCaseSensitive() const
{
  return caseSensitiveCheckBox_ctl->isChecked();
}

bool QFindTextDialog::isWholeWords() const
{
  return wholeWordsCheckBox_ctl->isChecked();
}

void QFindTextDialog::showEvent(QShowEvent * e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QFindTextDialog::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}
