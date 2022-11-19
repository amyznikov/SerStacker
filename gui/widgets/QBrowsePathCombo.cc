/*
 * QBrowsePathCombo.cc
 *
 *  Created on: Jan 7, 2017
 *      Author: amyznikov
 */

#include <gui/widgets/QBrowsePathCombo.h>

///////////////////////////////////////////////////////////////////////////////

QBrowsePathCombo::QBrowsePathCombo(QWidget *parent)
  : Base(parent)
{
  construct();
}

QBrowsePathCombo :: QBrowsePathCombo(const QString & label_, QFileDialog::FileMode mode, QWidget *parent)
  : Base(parent),
  fileDialogCaption(label_),
  labelText_(label_),
  fileDialogMode(mode)
{
  construct();
}

void QBrowsePathCombo::setShowDirsOnly(bool v)
{
  showDirsOnly_ = v;
}

bool QBrowsePathCombo::showDirsOnly() const
{
  return showDirsOnly_;
}

//setOption(ShowDirsOnly, true)


void QBrowsePathCombo::construct(void)
{
  QVBoxLayout * vbox;
  QHBoxLayout * hbox;

  vbox = new QVBoxLayout(this);
  vbox->addWidget(label = new QLabel(labelText_), 0, Qt::AlignLeft);
  vbox->addLayout(hbox = new QHBoxLayout(), 0);
  hbox->setContentsMargins(0,0,0,0);

#if !QT_DEPRECATED_SINCE(5, 13)
  hbox->setMargin(0);
#endif

  hbox->addWidget(combo = new QComboBox(this), -1);
  hbox->addWidget(button = new QToolButton(this), -1);

  combo->setEditable(true);
  combo->setDuplicatesEnabled(false);
  combo->setMinimumContentsLength(24);
  combo->setMaxCount(30);

  button->setText(tr("Browse..."));

  connect(button, SIGNAL(clicked(bool)),
      this, SLOT(onBrowseForPath()),
      Qt::QueuedConnection);

  connect(combo, SIGNAL(currentTextChanged(const QString &)),
      this, SLOT(currentTextChanged(const QString &)),
      Qt::DirectConnection);

//  connect(combo, SIGNAL(editTextChanged(const QString &)),
//      this, SLOT(editTextChanged(const QString &)),
//      Qt::DirectConnection);
}


void QBrowsePathCombo::setFileDialogCaption(const QString & caption)
{
  this->fileDialogCaption = caption;
}

void QBrowsePathCombo::setFileDialogMode(QFileDialog::FileMode mode)
{
  this->fileDialogMode = mode;
}

void QBrowsePathCombo::onBrowseForPath(void)
{
  QString path = combo->currentText();
  QFileInfo fileInfo(path);


//  if ( path.isEmpty() ) {
//    path = "/";
//  }

  QFileDialog dlg(this,
      fileDialogCaption.isEmpty() ? labelText_ : fileDialogCaption,
      fileInfo.isDir() ? path : fileInfo.filePath());

  dlg.setFileMode(fileDialogMode);
  dlg.setOption(QFileDialog::ShowDirsOnly, showDirsOnly_);

  //dlg.setOption(QFileDialog::DontUseNativeDialog);
  dlg.setViewMode(fileDialogViewMode);
  dlg.selectFile(path);


  //  QSortFilterProxyModel *sorter = new QSortFilterProxyModel();
  //  sorter->setDynamicSortFilter(true); // This ensures the proxy will resort when the model changes
  //  dlg.setProxyModel(sorter);

  if ( dlg.exec() == QDialog::Accepted ) {

    if ( dlg.options() & QFileDialog::ShowDirsOnly /*fileDialogMode == QFileDialog::DirectoryOnly*/ ) {
      path = dlg.directory().path();
    }
    else {
      QStringList selectedFiles = dlg.selectedFiles();
      if ( selectedFiles.size() > 0 ) {
        path = selectedFiles[0];
      }
    }
    addPath(path, true);
    emit pathSelected(path);
  }

  fileDialogViewMode = dlg.viewMode(); // save user preference
}


void QBrowsePathCombo::addPath(const QString & path, bool emitHasChages)
{
  enableEmitChagesEvent_ = emitHasChages;

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

  enableEmitChagesEvent_ = true;
}

void QBrowsePathCombo::setCurrentPath(const QString & path, bool emitHasChages)
{
  enableEmitChagesEvent_ = emitHasChages;
  combo->setCurrentText(path);
  if ( !emitHasChages ) {
    setHasChanges(false);
  }
  enableEmitChagesEvent_ = true;
}

QString QBrowsePathCombo::currentPath(void) const
{
  return combo->currentText();
}

bool QBrowsePathCombo::hasChanges(void) const
{
  return hasChanges_;
}

void QBrowsePathCombo::setHasChanges(bool f)
{
  hasChanges_ = f;
}

void QBrowsePathCombo::currentTextChanged(const QString &)
{
  if ( enableEmitChagesEvent_ ) {
    hasChanges_ = true;
    emit pathChanged();
  }
}

///////////////////////////////////////////////////////////////////////////////
