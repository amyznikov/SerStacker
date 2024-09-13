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

QBrowsePathCombo::QBrowsePathCombo(const QString & label_, QFileDialog::AcceptMode acceptMode,
    QFileDialog::FileMode mode, QWidget * parent) :
    Base(parent),
    fileDialogCaption(label_),
    _labelText(label_),
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

  connect(button, SIGNAL(clicked(bool)),
      this, SLOT(onBrowseForPath()),
      Qt::QueuedConnection);

  connect(combo, SIGNAL(currentTextChanged(const QString &)),
      this, SLOT(currentTextChanged(const QString &)),
      Qt::DirectConnection);
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
    addPath(path, true);
    Q_EMIT pathSelected(path);
  }
}


void QBrowsePathCombo::addPath(const QString & path, bool emitHasChages)
{
  _enableEmitChagesEvent = emitHasChages;

  int existing_index = combo->findText(path); // check if item exists
  if ( existing_index < 0 ) {
    combo->insertItem(0, path);
  }
  else if ( existing_index > 0 ) { // not exists
    combo->removeItem(existing_index);
    combo->insertItem(0, path);
  }
  combo->setCurrentIndex(0);

  if ( !emitHasChages ) {
    setHasChanges(false);
  }

  _enableEmitChagesEvent = true;
}

void QBrowsePathCombo::setCurrentPath(const QString & path, bool emitHasChages)
{
  _enableEmitChagesEvent = emitHasChages;
  combo->setCurrentText(path);
  if ( path.isEmpty() ) {
    combo->setCurrentIndex(-1);
  }
  if ( !emitHasChages ) {
    setHasChanges(false);
  }
  _enableEmitChagesEvent = true;
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

void QBrowsePathCombo::currentTextChanged(const QString &)
{
  if ( _enableEmitChagesEvent ) {
    _hasChanges = true;
    emit pathChanged();
  }
}

///////////////////////////////////////////////////////////////////////////////
