/*
 * QTextInfoDialogBox.cc
 *
 *  Created on: Apr 4, 2026
 *      Author: amyznikov
 */

#include "QTextInfoDialogBox.h"

QTextInfoDialogBox * QTextInfoDialogBox::_singleton = nullptr;

QTextInfoDialogBox::QTextInfoDialogBox(QWidget * parent) :
  Base(parent)
{
  QVBoxLayout * vbox = new QVBoxLayout(this);

  vbox->addWidget(toolbar_ctl = new QToolBar(this));
  vbox->addWidget(textbox_ctl = new QTextBrowser(this));

  toolbar_ctl->addWidget(new QLabel("Find: ", this));
  toolbar_ctl->addWidget(searchText_ctl = new QLineEditBox(this));

  QFont font("Monospace");
  font.setStyleHint(QFont::Monospace);
  textbox_ctl->document()->setDefaultFont(font);

  QPalette p = textbox_ctl->palette();
  p.setColor(QPalette::Highlight, Qt::blue);
  p.setColor(QPalette::HighlightedText, Qt::white);
  textbox_ctl->setPalette(p);

  textbox_ctl->setWordWrapMode(QTextOption::WrapMode::NoWrap);
  textbox_ctl->setReadOnly(true);
  textbox_ctl->setUndoRedoEnabled(false);

  connect(searchText_ctl, &QLineEditBox::textChanged,
      [this]() {
        searchNext_ctl->setEnabled(!searchText_ctl->text().isEmpty());
      });

  toolbar_ctl->addAction(searchNext_ctl =
      createAction(QIcon(), "Find Next",
          "Find next occurrence of text",
          [this]() {
            const QString text = searchText_ctl->text();
            if ( !text.isEmpty() && !textbox_ctl->find(text) ) {
              textbox_ctl->moveCursor(QTextCursor::Start);
              textbox_ctl->find(text);
            }
          },
          new QShortcut(QKeySequence::FindNext, this)));

  searchNext_ctl->setEnabled(!searchText_ctl->text().isEmpty());

  connect(searchText_ctl, &QLineEditBox::returnPressed,
      searchNext_ctl, &QAction::trigger);


}

void QTextInfoDialogBox::hideEvent(QHideEvent * event)
{
  _geometry = saveGeometry();
  Base::hideEvent(event);
}

void QTextInfoDialogBox::show(const QString & title, const QString & text, QWidget * parent)
{
  if( !_singleton ) {
    _singleton = new ThisClass(parent ? parent : QApplication::activeWindow());
  }

  _singleton->setWindowTitle(title);
  _singleton->textbox_ctl->setText(text);
  if( !_singleton->_geometry.isEmpty() ) {
    _singleton->restoreGeometry(_singleton->_geometry);
  }

  _singleton->textbox_ctl->moveCursor(QTextCursor::Start);
  _singleton->textbox_ctl->ensureCursorVisible();

  _singleton->Base::show();
  _singleton->raise();
  _singleton->activateWindow();
}
