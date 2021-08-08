/*
 * QStackingSettingsWidget.cc
 *
 *  Created on: Feb 8, 2021
 *      Author: amyznikov
 */

#include "QStackOptions.h"
#include <gui/qstackingthread/QStackingThread.h>
#include <gui/widgets/addctrl.h>

#define ICON_close          "close"
#define ICON_check_all      "check_all"

static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qstackingoptions/icons/%1").arg(name));
}


QStackingSettingsWidget::QStackingSettingsWidget(QWidget * parent)
  : Base("QStackingSettingsWidget", parent)
{
  Q_INIT_RESOURCE(qstackingoptions_resources);

  stackName_ctl =
      add_editbox(form, "* Stack Name:",
          [this]() {
            if ( stack_ && !updatingControls() ) {
              const QString & text = stackName_ctl->text();
              if ( !text.isEmpty() ) {
                LOCK(); stack_->set_name(text.toStdString()); UNLOCK();
                emit stackNameChanged(stack_);
              }
            }
          });


  add_expandable_groupbox(form, "* Input Options",
      inputOptions_ctl =
          new QImageStackingInputOptions(this));

  add_expandable_groupbox(form, "* Master Frame Options",
      masterFrame_ctl =
          new QMasterFrameOptions(this));

  add_expandable_groupbox(form, "* ROI Selection",
      roiSelection_ctl =
          new QROISelectionOptions(this));

  add_expandable_groupbox(form, "* Frame Accumulation Options",
      frameAccumulation_ctl =
          new QFrameAccumulationOptions(this));

  add_expandable_groupbox(form,
      "* Frame Registration Options",
      frameRegistration_ctl =
          new QFrameRegistrationOptions(this));

  add_expandable_groupbox(form,
      "* Output Options",
      outputOptions_ctl =
          new QStackOutputOptions(this));


  connect(inputOptions_ctl, &QImageStackingInputOptions::applyInputOptionsToAllRequested,
      this, &ThisClass::applyInputOptionsToAllRequested);

  connect(masterFrame_ctl, &QMasterFrameOptions::applyMasterFrameSettingsToAllRequested,
      this, &ThisClass::applyMasterFrameOptionsToAllRequested);

  connect(roiSelection_ctl, &QROISelectionOptions::applyROISelectionOptionsToAllRequested,
      this, &ThisClass::applyROISelectionOptionsToAllRequested);

  connect(frameAccumulation_ctl, &QFrameAccumulationOptions::applyFrameAccumulationOptionsToAllRequested,
      this, &ThisClass::applyFrameAccumulationOptionsToAllRequested);

  connect(frameRegistration_ctl, &QFrameRegistrationOptions::applyFrameRegistrationOptionsToAllRequested,
      this, &ThisClass::applyFrameRegistrationOptionsToAllRequested);

  connect(outputOptions_ctl, &QStackOutputOptions::applyOutputOptionsToAllRequested,
      this, &ThisClass::applyOutputOptionsToAllRequested);


  updateControls();
}

void QStackingSettingsWidget::setCurrentStack(const c_image_stacking_options::ptr & options)
{
  stack_ = options;
  updateControls();
}

const c_image_stacking_options::ptr & QStackingSettingsWidget::currentStack() const
{
  return stack_;
}

void QStackingSettingsWidget::onupdatecontrols()
{
  if ( !stack_ ) {
    setEnabled(false);

    stackName_ctl->setText("");
    inputOptions_ctl->set_input_options(nullptr);
    masterFrame_ctl->set_master_frame_options(nullptr, nullptr);
    roiSelection_ctl->set_roi_selection_options(nullptr);
    frameAccumulation_ctl->set_accumulation_options(nullptr);
    frameRegistration_ctl->set_registration_options(nullptr);
    outputOptions_ctl->set_output_options(nullptr);

  }
  else {

    stackName_ctl->setText(stack_->name().c_str());
    inputOptions_ctl->set_input_options(&stack_->input_options());
    masterFrame_ctl->set_master_frame_options(&stack_->master_frame_options(), stack_->input_sequence());
    roiSelection_ctl->set_roi_selection_options(&stack_->roi_selection_options());
    frameAccumulation_ctl->set_accumulation_options(&stack_->accumulation_options());
    frameRegistration_ctl->set_registration_options(&stack_->frame_registration_options());
    outputOptions_ctl->set_output_options(&stack_->output_options());

    if ( QStackingThread::isRunning() && stack_ == QStackingThread::currentStack() ) {
      setEnabled(false);
    }
    else {
      setEnabled(true);
    }
  }
}



QStackOptions::QStackOptions(QWidget * parent)
  : Base(parent)
{

  QAction * action;

  Q_INIT_RESOURCE(qstackingoptions_resources);

  static const auto createScrollableWrap =
      [](QWidget * w, QWidget * parent = Q_NULLPTR) -> QScrollArea *
  {
    QScrollArea * scrollArea = new QScrollArea(parent ? parent : w->parentWidget());
    scrollArea->setWidgetResizable(true);
    scrollArea->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setWidget(w);
    return scrollArea;
  };

  static const auto addSpacer =
      [](QToolBar * toolBar) -> QWidget *
  {
    QWidget * spacer = new QWidget();
    spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    toolBar->addWidget(spacer);
    return spacer;
  };


  layout_ = new QVBoxLayout(this);
  toolbar_ = new QToolBar(this);
  toolbar_->setIconSize(QSize(16, 16));
  toolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
  toolbar_->setOrientation(Qt::Horizontal);

  toolbar_->addWidget(new QLabel("<strong>Stacking options</strong>"));

  addSpacer(toolbar_);

  toolbar_->addAction(action = new QAction(getIcon(ICON_check_all), "Apply to all..."));
  action->setToolTip("Set these parameters to all selected stacks");
  connect(action, &QAction::triggered,
      [this]() {
        if ( stackSettings_ctl->currentStack() ) {
          emit applyAllStackOptionsToAllRequested(stackSettings_ctl->currentStack());
        }
      });


  toolbar_->addAction(action = new QAction(getIcon(ICON_close), "Close"));
  action->setShortcut(QKeySequence::Cancel);
  action->setToolTip("Close window");
  connect(action, &QAction::triggered, this,
      &ThisClass::closeWindowRequested);



  stackSettings_ctl =  new QStackingSettingsWidget(this);
  scrollArea_ = createScrollableWrap(stackSettings_ctl);


  layout_->addWidget(toolbar_, 1, Qt::AlignTop);
  layout_->addWidget(scrollArea_, 100);

  connect(stackSettings_ctl, &QStackingSettingsWidget::applyInputOptionsToAllRequested,
      this, &ThisClass::applyInputOptionsToAllRequested);

  connect(stackSettings_ctl, &QStackingSettingsWidget::applyMasterFrameOptionsToAllRequested,
      this, &ThisClass::applyMasterFrameOptionsToAllRequested);

  connect(stackSettings_ctl, &QStackingSettingsWidget::applyROISelectionOptionsToAllRequested,
      this, &ThisClass::applyROISelectionOptionsToAllRequested);

  connect(stackSettings_ctl, &QStackingSettingsWidget::applyFrameAccumulationOptionsToAllRequested,
      this, &ThisClass::applyFrameAccumulationOptionsToAllRequested);

  connect(stackSettings_ctl, &QStackingSettingsWidget::applyFrameRegistrationOptionsToAllRequested,
      this, &ThisClass::applyFrameRegistrationOptionsToAllRequested);

  connect(stackSettings_ctl, &QStackingSettingsWidget::applyOutputOptionsToAllRequested,
      this, &ThisClass::applyOutputOptionsToAllRequested);

}

void QStackOptions::setCurrentStack(const c_image_stacking_options::ptr & options)
{
  stackSettings_ctl->setCurrentStack(options);
}

const c_image_stacking_options::ptr & QStackOptions::currentStack() const
{
  return stackSettings_ctl->currentStack();
}


void QStackOptions::updateControls()
{
  return stackSettings_ctl->updateControls();
}

