/*
 * QTextFileViewer.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#include "QTextFileViewer.h"
#include <gui/widgets/QWaitCursor.h>

QTextFileViewer::QTextFileViewer(QWidget * parent)
  : Base(parent)
{
  layout_ = new QVBoxLayout(this);
  layout_->setContentsMargins(0, 0, 0, 0);

  toolbar_ = new QToolBar(this);
  toolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
  toolbar_->setOrientation(Qt::Horizontal);
  toolbar_->setIconSize(QSize(16,16));

  textBrowser_ = new QTextEdit(this);
  textBrowser_->setReadOnly(true);
  textBrowser_->setAutoFormatting(QTextBrowser::AutoAll);

  layout_->addWidget(toolbar_, 1);
  layout_->addWidget(textBrowser_, 100);

}

QToolBar * QTextFileViewer::toolbar() const
{
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
  emit currentFileNameChanged();
}

void QTextFileViewer::showTextFile(const std::string & pathfilename)
{
  showTextFile(QString::fromStdString(pathfilename));
}

void QTextFileViewer::showTextFile(const QString & pathfilename)
{

  textBrowser_->clear();

  if ( !(currentFileName_ = pathfilename).isEmpty() ) {

    QWaitCursor wait(this);
    QFile file(currentFileName_);

    if ( file.open(QIODevice::ReadOnly | QIODevice::Text) ) {
      textBrowser_->setText(file.readAll());
    }

  }

  emit currentFileNameChanged();
}
