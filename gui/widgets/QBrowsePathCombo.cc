/*
 * QBrowsePathCombo.cc
 *
 *  Created on: Jan 7, 2017
 *      Author: amyznikov
 */

#include <gui/widgets/QBrowsePathCombo.h>
#include <gui/widgets/style.h>
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////

QBrowsePathCombo::QBrowsePathCombo(QWidget *parent) :
  Base(parent)
{
  construct();
}

QBrowsePathCombo::QBrowsePathCombo(const QString & label, QFileDialog::AcceptMode acceptMode,
    QFileDialog::FileMode mode, QWidget * parent) :
    Base(parent),
    fileDialogCaption(label),
    _labelText(label),
    _fileMode(mode),
    _acceptMode(acceptMode)
{
  construct();
}

void QBrowsePathCombo::setShowDirsOnly(bool v)
{
  _showDirsOnly = v;
}

bool QBrowsePathCombo::showDirsOnly() const
{
  return _showDirsOnly;
}

void QBrowsePathCombo::construct(void)
{
  QVBoxLayout * vbox;
  QHBoxLayout * hbox;

  setContentsMargins(0, 0, 0, 0);

  vbox = new QVBoxLayout(this);
  vbox->setContentsMargins(0,0,0,0);

  if ( !_labelText.isEmpty() ) {
    vbox->addWidget(label = new QLabel(_labelText), 0, Qt::AlignLeft);
  }

  vbox->addLayout(hbox = new QHBoxLayout(), 0);
  hbox->setContentsMargins(0,0,0,0);

#if !QT_DEPRECATED_SINCE(5, 13)
  hbox->setMargin(0);
#endif

  hbox->addWidget(combo = new QComboBox(this), 1000);
  hbox->addWidget(button = new QToolButton(this), 1);

  combo->setEditable(true);
  combo->setDuplicatesEnabled(false);
  combo->setFocusPolicy(Qt::StrongFocus);
  combo->setMinimumContentsLength(16);
  combo->setMaxCount(30);

  if( _acceptMode == QFileDialog::AcceptSave ) {
    combo->lineEdit()->setPlaceholderText("auto");
#if QT_VERSION >= QT_VERSION_CHECK(5, 16, 0)
    combo->setPlaceholderText("auto");
#endif
  }

  button->setText(tr("Browse..."));

  connect(button, &QToolButton::clicked,
      this, &ThisClass::onBrowseForPath,
      Qt::QueuedConnection);

  QLineEdit* edit = combo->lineEdit();
  QObject::connect(edit, &QLineEdit::textEdited, this, [this]() {
    _hasChanges = true;
  });
  QObject::connect(combo, QOverload<int>::of(&QComboBox::activated), this, [this](int /*index*/) {
     _hasChanges = false;
     Q_EMIT pathChanged();
 });
  QObject::connect(edit, &QLineEdit::returnPressed, this, [this]() {
   if (_hasChanges) {
     _hasChanges = false;
     Q_EMIT pathChanged();
   }
 });
 combo->installEventFilter(this);
}

bool QBrowsePathCombo::eventFilter(QObject * watched, QEvent * event)
{
  if( _hasChanges && combo && watched == combo ) {
    if( event->type() == QEvent::FocusOut ) {
      _hasChanges = false;
      Q_EMIT pathChanged();
    }
  }
  return Base::eventFilter(watched, event);
}

void QBrowsePathCombo::setFileDialogCaption(const QString & caption)
{
  this->fileDialogCaption = caption;
}

void QBrowsePathCombo::setFileMode(QFileDialog::FileMode mode)
{
  _fileMode = mode;
}

QFileDialog::FileMode QBrowsePathCombo::fileMode() const
{
  return _fileMode;
}

void QBrowsePathCombo::setAcceptMode(QFileDialog::AcceptMode mode)
{
  _acceptMode = mode;
}

QFileDialog::AcceptMode QBrowsePathCombo::acceptMode() const
{
  return _acceptMode;
}

void QBrowsePathCombo::setFileFilter(const QString & v)
{
  _fileFilter = v;
}

const QString & QBrowsePathCombo::fileFilter() const
{
  return _fileFilter;
}


void QBrowsePathCombo::onBrowseForPath(void)
{
  QString title = fileDialogCaption.isEmpty() ? _labelText : fileDialogCaption;
  QString path = combo->currentText();

  switch (_fileMode) {
    case QFileDialog::Directory: {
      QFileInfo fileInfo(path);
      QString dir = fileInfo.isDir() ? path : fileInfo.filePath();
      path = QFileDialog::getExistingDirectory(this, title, dir, QFileDialog::DontUseNativeDialog);
      break;
    }

    default:
      switch (_acceptMode) {
        case QFileDialog::AcceptOpen:
          path = QFileDialog::getOpenFileName(this,
              title,
              path,
              _fileFilter,
              nullptr,
              QFileDialog::DontUseNativeDialog);
          break;

        case QFileDialog::AcceptSave:
          path = QFileDialog::getSaveFileName(this,
              title,
              path,
              _fileFilter,
              nullptr,
              QFileDialog::DontUseNativeDialog);
          break;

        default:
          path.clear();
          break;
      }
      break;
  }

  if( !path.isEmpty() ) {
    addPath(path);
  }
}


void QBrowsePathCombo::addPath(const QString & path)
{
  // QSignalBlocker block(this);

  int existing_index = combo->findText(path); // check if item exists
  if ( existing_index < 0 ) {
    combo->insertItem(0, path);
  }
  else if ( existing_index > 0 ) { // not exists
    combo->removeItem(existing_index);
    combo->insertItem(0, path);
  }
  combo->setCurrentIndex(0);
  _hasChanges = false;
  // Q_EMIT pathChanged();
}

void QBrowsePathCombo::setCurrentPath(const QString & path)
{
  QSignalBlocker block(combo);
  combo->setCurrentText(path);
  if ( path.isEmpty() ) {
    combo->setCurrentIndex(-1);
  }
  setHasChanges(false);
}

QString QBrowsePathCombo::currentPath(void) const
{
  return combo->currentText();
}

bool QBrowsePathCombo::hasChanges(void) const
{
  return _hasChanges;
}

void QBrowsePathCombo::setHasChanges(bool f)
{
  _hasChanges = f;
}

//void QBrowsePathCombo::currentTextChanged(const QString &)
//{
//  if ( _enableEmitChagesEvent ) {
//    _hasChanges = true;
//    Q_EMIT pathChanged();
//  }
//}

///////////////////////////////////////////////////////////////////////////////
