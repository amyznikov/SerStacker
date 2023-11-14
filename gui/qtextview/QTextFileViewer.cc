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
  layout_ = new QVBoxLayout(this);
  layout_->setContentsMargins(0, 0, 0, 0);

//  toolbar_ = new QToolBar(this);
//  toolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
//  toolbar_->setOrientation(Qt::Horizontal);
//  toolbar_->setIconSize(QSize(16,16));

  textBrowser_ = new QPlainTextEdit(this);
  textBrowser_->setReadOnly(true);
  textBrowser_->setLineWrapMode(QPlainTextEdit::NoWrap);
  textBrowser_->setWordWrapMode(QTextOption::WrapMode::NoWrap);
  textBrowser_->setFont(QFont("monospace"));
  //textBrowser_->setAutoFormatting(QTextBrowser::AutoNone);

  //layout_->addWidget(toolbar_, 1);
  layout_->addWidget(textBrowser_, 100);

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
  if ( !toolbar_ ) {
    toolbar_ = new QToolBar(this);
    toolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
    toolbar_->setOrientation(Qt::Horizontal);
    toolbar_->setIconSize(QSize(16,16));
    layout_->insertWidget(0, toolbar_, 1);
  }
  return toolbar_;
}

QString QTextFileViewer::currentFileName() const
{
  return currentFileName_;
}

void QTextFileViewer::clear()
{
  textBrowser_->clear();
  currentFileName_.clear();
  Q_EMIT currentFileNameChanged();
}

void QTextFileViewer::showTextFile(const std::string & pathfilename)
{
  showTextFile(QString::fromStdString(pathfilename));
}

void QTextFileViewer::showTextFile(const QString & pathfilename)
{

  textBrowser_->clear();

  if ( !(currentFileName_ = pathfilename).isEmpty() ) {

    // QWaitCursor wait(this);

    QFile file(currentFileName_);

    if ( file.open(QIODevice::ReadOnly | QIODevice::Text) ) {
      textBrowser_->setPlainText(file.readAll());
    }

  }

  Q_EMIT currentFileNameChanged();
}
