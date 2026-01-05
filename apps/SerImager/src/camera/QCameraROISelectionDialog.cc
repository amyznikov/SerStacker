/*
 * QCameraROISelectionDialog.cc
 *
 *  Created on: Oct 28, 2023
 *      Author: amyznikov
 */

#include "QCameraROISelectionDialog.h"
#include <core/ssprintf.h>
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

bool QCameraROI::metatype_registered_ =
    QCameraROI::registerMetatype();

///////////////////////////////////////////////////////////////////////////////////////////////////

template<class Fn>
static QToolButton * createToolButton(QWidget * parent,
    const QIcon & icon, const QString & text, const QString & tooltip,
    Fn && clickFunc)
{

  QToolButton * tb = new QToolButton(parent);
  tb->setIconSize(QSize(16, 16));
  tb->setIcon(icon);
  tb->setText(text);
  tb->setToolTip(tooltip);

  QObject::connect(tb, &QToolButton::clicked,
      clickFunc);

  return tb;
}



///////////////////////////////////////////////////////////////////////////////////////////////////

bool QCameraROI::registerMetatype()
{
  if( !metatype_registered_ ) {
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    qRegisterMetaTypeStreamOperators<QCameraROI>("QCameraROI");
    qRegisterMetaTypeStreamOperators<QList<QCameraROI>>("QList<QCameraROI>");
#endif
    metatype_registered_ = true;
  }
  return metatype_registered_;
}

QCameraROISelectionDialog::QCameraROISelectionDialog(QWidget * parent) :
  Base(parent)
{
  setWindowTitle("Define Custom ROI");

  QHBoxLayout * hbox0 =
      new QHBoxLayout(this);

  QVBoxLayout * vboxl =
      new QVBoxLayout();

  QVBoxLayout * vboxr =
      new QVBoxLayout();

  vboxr->setAlignment(Qt::AlignTop);


  /////////////////////////////////////////////////////////////////////////
  vboxl->addWidget(roiOptions_ctl =
      new QSettingsWidget("", this));

  roiSelection_ctl =
      roiOptions_ctl->add_combobox<QComboBox>("ROI:", "Select ROI to edit ",
          false,
          [this](int index, QComboBox * combo) {

            updatingControls_ = true;

            addRoi_ctl->setEnabled(false);

            if ( index < 0 ) {
              deleteRoi_ctl->setEnabled(false);
            }
            else {

              const QCameraROI & roi =
                  roiList_[index];

              deleteRoi_ctl->setEnabled(true);

              roiName_ctl_->setText(roi.name);
              roiRect_ctl->setText(toQString(roi.rect));
            }

            updatingControls_ = false;
          });

  roiSelection_ctl->setSizeAdjustPolicy(QComboBox::AdjustToContents);

  roiName_ctl_ =
      roiOptions_ctl->add_textbox("ROI Name:",
          "ROI name",
          [this](const QString & name) {
            if ( !updatingControls_ ) {
              if ( name.isEmpty() || findROIByName(roiList_, name) >= 0 ) {
                addRoi_ctl->setEnabled(false);
              }
              else {
                addRoi_ctl->setEnabled(true);
              }
              setHasChanges(true);
            }
          });

  roiRect_ctl =
      roiOptions_ctl->add_numeric_box<QRect>("ROI rect:",
          "Specify ROI rectangle.\n"
              "Use ROI selection tool button and context menu to copy and paste ROI text",
          [this](const QRect & rc) {

            if ( !updatingControls_ && !rc.isEmpty() ) {

              const QString name =
                  roiName_ctl_->text();

              if ( !name.isEmpty() ) {

                int index =
                    findROIByName(roiList_, name);

                if ( index >= 0 && index < roiList_.size() ) {
                  roiList_[index].rect = rc;
                  updatingControls_ = true;
                  roiSelection_ctl->setItemText(index, roiList_[index].toQString());
                  setHasChanges(true);
                  updatingControls_ = false;
                }
              }
            }
          });

  // QCameraROI
  /////////////////////////////////////////////////////////////////////////

  vboxr->addWidget(addRoi_ctl =
      createToolButton(this, QIcon(),
          "Add ROI",
          "",
          [this]() {
            const QString name = roiName_ctl_->text();
            if ( !name.isEmpty() ) {
              int index = findROIByName(roiList_, name);
              if ( index >= 0 ) {
                roiSelection_ctl->setCurrentIndex(index);
              }
              else {
                QRect rc;
                if ( fromString(roiRect_ctl->text(), &rc) && (!validator_ || validator_(rc)) ) {
                  roiList_.append(QCameraROI(name, rc ));
                  roiSelection_ctl->addItem(roiList_[index = roiList_.size()-1].toQString());
                  roiSelection_ctl->setCurrentIndex(index);
                  setHasChanges(true);
                }
              }
            }
          }));

  vboxr->addWidget(deleteRoi_ctl =
      createToolButton(this, QIcon(),
          "Delete ROI",
          "",
          [this]() {
            const QString name = roiName_ctl_->text();
            if ( !name.isEmpty() ) {
              int index = findROIByName(roiList_, name);
              if ( index >= 0 ) {
                roiList_.removeAt(index);
                roiSelection_ctl->removeItem(index);
                setHasChanges(true);
              }
            }
          }));

  /////////////////////////////////////////////////////////////////////////

  hbox0->addLayout(vboxl);
  hbox0->addLayout(vboxr);

  populateROISelectionCombo();
}

void QCameraROISelectionDialog::setROIList(const QList<QCameraROI> & list)
{
  roiList_ = list;
  populateROISelectionCombo();
}

const QList<QCameraROI> & QCameraROISelectionDialog::roiList() const
{
  return roiList_;
}

void QCameraROISelectionDialog::populateROISelectionCombo()
{
  updatingControls_ = true;
  roiSelection_ctl->clear();

  for( int i = 0, n = roiList_.size(); i < n; ++i ) {
    roiSelection_ctl->addItem(roiList_[i].toQString());
  }

  updatingControls_ = false;

  if ( roiSelection_ctl->count() > 0 ) {
    roiSelection_ctl->setCurrentIndex(0);
  }

}


void QCameraROISelectionDialog::setHasChanges(bool v)
{
  hasChanges_ = v;
}

bool QCameraROISelectionDialog::hasChanges() const
{
  return hasChanges_;
}


int QCameraROISelectionDialog::findROIByName(const QList<QCameraROI> & list, const QString & name)
{
  for( int i = 0, n = list.size(); i < n; ++i ) {
    if ( list[i].name == name ) {
      return i;
    }
  }
  return -1;
}

void QCameraROISelectionDialog::setValidator(const ROIValidator & v)
{
  validator_ = v;
}

const QCameraROISelectionDialog::ROIValidator & QCameraROISelectionDialog::validator() const
{
  return validator_;
}

