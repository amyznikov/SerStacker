/*
 * QPipelineOptionsView.cc
 *
 *  Created on: Feb 22, 2023
 *      Author: amyznikov
 */

#include "QPipelineOptionsView.h"
#include <gui/qpipelinethread/QImageProcessingPipeline.h>
#include <gui/widgets/qsprintf.h>
#include <gui/widgets/style.h>
#include <core/ssprintf.h>

#define ICON_close          ":/qpipelineoptions/icons/close"
#define ICON_menu           ":/qpipelineoptions/icons/menu"

QPipelineOptionsView::QPipelineOptionsView(QWidget * parent) :
  Base(parent)
{

  Q_INIT_RESOURCE(qpipelineoptions_resources);

  static const auto addSpacer =
      [](QToolBar * toolBar) -> QWidget *
  {
    QWidget * spacer = new QWidget();
    spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    toolBar->addWidget(spacer);
    return spacer;
  };


  layout_ = new QVBoxLayout(this);

  toolbar_ = new QToolBar();
  toolbar_->setIconSize(QSize(16, 16));

  toolbar_->addWidget(sequenceName_lb = new QLabel(""));
  sequenceName_lb->setTextFormat(Qt::RichText);
  toolbar_->addSeparator();

  toolbar_->addWidget(pipelineSelector_ctl = new QComboBox(this));
  pipelineSelector_ctl->setEditable(false);
  pipelineSelector_ctl->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  pipelineSelector_ctl->setToolTip("Select current image processing pipeline");
  connect(pipelineSelector_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      this, &ThisClass::onPipelineSelectorCurrentIndexChanged);


  toolbar_->addWidget(menuButton = new QToolButton());
  menuButton->setText("Options...");
  menuButton->setIcon(getIcon(ICON_menu));
  menuButton->setToolTip("Add / Remove image processing pipelines");
  menuButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  connect(menuButton, &QToolButton::clicked,
      this, &ThisClass::onMenuButtonClicked);

  toolbar_->addSeparator();

  addSpacer(toolbar_);

  toolbar_->addAction(close_ctl = new QAction(getIcon(ICON_close), "Close"));
  close_ctl->setShortcut(QKeySequence::Cancel);
  close_ctl->setToolTip("Close window");
  connect(close_ctl, &QAction::triggered, this,
      &ThisClass::closeWindowRequested);


  scrollArea_ctl = new QScrollArea(this);
  scrollArea_ctl->setWidgetResizable(true);
  scrollArea_ctl->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
  scrollArea_ctl->setFrameShape(QFrame::NoFrame);


  layout_->addWidget(toolbar_, 1, Qt::AlignTop);
  layout_->addWidget(scrollArea_ctl, 1000);



  imageStackingOptions_ctl = new QImageStackingOptions(this);
  imageStackingOptions_ctl->setVisible(false);
  connect(imageStackingOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);


  cameraCalibrationOptions_ctl = new QCameraCalibrationPipelineOptions(this);
  cameraCalibrationOptions_ctl->setVisible(false);
  connect(cameraCalibrationOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  stereoCalibrationOptions_ctl = new QStereoCalibrationPipelineOptions(this);
  stereoCalibrationOptions_ctl->setVisible(false);
  connect(stereoCalibrationOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  rstereoCalibrationOptions_ctl = new QRStereoOptions(this);
  rstereoCalibrationOptions_ctl->setVisible(false);
  connect(rstereoCalibrationOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);
}

void QPipelineOptionsView::set_current_sequence(const c_image_sequence::sptr & image_sequence)
{
  updatingControls_ = true;
  pipelineSelector_ctl->clear();

  if( !(current_sequence_ = image_sequence) ) {
    enablecontrols(false);
    sequenceName_lb->setText("");
    updateCurrentSettingsWidget(nullptr);
  }
  else {

    sequenceName_lb->setText(qsprintf("<strong>%s</strong>",
        current_sequence_->name().c_str()));

    const std::vector<c_image_processing_pipeline::sptr> &pipelines =
        current_sequence_->pipelines();

    for( const c_image_processing_pipeline::sptr &pipeline : pipelines ) {
      pipelineSelector_ctl->addItem(pipeline->name().c_str());
    }

    const c_image_processing_pipeline::sptr &current_pipeline =
        current_sequence_->current_pipeline();

    if( !current_pipeline ) {
      pipelineSelector_ctl->setCurrentIndex(-1);
      updateCurrentSettingsWidget(nullptr);
    }
    else {

      int index = pipelineSelector_ctl->findText(current_pipeline->name().c_str());
      if( index < 0 ) {
        pipelineSelector_ctl->addItem(current_pipeline->name().c_str());
        index = pipelineSelector_ctl->findText(current_pipeline->name().c_str());
      }

      pipelineSelector_ctl->setCurrentIndex(index);
      updateCurrentSettingsWidget(current_pipeline);
    }

    enablecontrols(true);
  }

  updatingControls_ = false;
}

const c_image_sequence::sptr & QPipelineOptionsView::current_sequence() const
{
  return current_sequence_;
}

void QPipelineOptionsView::updateCurrentSettingsWidget(const c_image_processing_pipeline::sptr &pipeline)
{
  QWidget *currentWidget = nullptr;

  if( pipeline ) {

    if( c_image_stacking_pipeline::sptr image_stacking =
        std::dynamic_pointer_cast<c_image_stacking_pipeline>(pipeline) ) {

      currentWidget = imageStackingOptions_ctl;
      imageStackingOptions_ctl->set_current_pipeline(image_stacking);
    }

    else if( c_camera_calibration_pipeline::sptr camera_calibration =
        std::dynamic_pointer_cast<c_camera_calibration_pipeline>(pipeline) ) {

      currentWidget = cameraCalibrationOptions_ctl;
      cameraCalibrationOptions_ctl->set_current_pipeline(camera_calibration);
    }

    else if( c_stereo_calibration_pipeline::sptr stereo_calibration =
        std::dynamic_pointer_cast<c_stereo_calibration_pipeline>(pipeline) ) {

      currentWidget = stereoCalibrationOptions_ctl;
      stereoCalibrationOptions_ctl->set_current_pipeline(stereo_calibration);
    }

    else if( c_regular_stereo_pipeline::sptr rstereo_calibration =
        std::dynamic_pointer_cast<c_regular_stereo_pipeline>(pipeline) ) {

      currentWidget = rstereoCalibrationOptions_ctl;
      rstereoCalibrationOptions_ctl->set_current_pipeline(rstereo_calibration);
    }
  }


  if( scrollArea_ctl->widget() != currentWidget ) {

    if( scrollArea_ctl->widget() ) {
      scrollArea_ctl->widget()->hide();
    }

    scrollArea_ctl->takeWidget();
    scrollArea_ctl->setWidget(currentWidget);

    if( scrollArea_ctl->widget() ) {
      scrollArea_ctl->widget()->show();
    }
  }

  if ( currentWidget ) {

    if ( QImageProcessingPipeline::isRunning() && pipeline == QImageProcessingPipeline::current_pipeline() ) {
      currentWidget->setEnabled(false);
    }
    else {
      currentWidget->setEnabled(true);
    }
  }

}

void QPipelineOptionsView::enablecontrols(bool v)
{
  menuButton->setEnabled(v);
  pipelineSelector_ctl->setEnabled(v && pipelineSelector_ctl->count() > 0);
}

void QPipelineOptionsView::oncurrentpipelinechanged()
{
  if( current_sequence_ ) {

    updatingControls_ = true;

    const c_image_processing_pipeline::sptr &current_pipeline =
        current_sequence_->current_pipeline();

    int index;

    if( !current_pipeline ) {
      index = -1;
    }
    else if( (index = pipelineSelector_ctl->findText(current_pipeline->name().c_str())) < 0 ) {
      pipelineSelector_ctl->addItem(current_pipeline->name().c_str());
      index = pipelineSelector_ctl->findText(current_pipeline->name().c_str());
    }

    pipelineSelector_ctl->setCurrentIndex(index);
    updateCurrentSettingsWidget(current_pipeline);

    enablecontrols(true);

    updatingControls_ = false;

    Q_EMIT parameterChanged();
  }
}


void QPipelineOptionsView::onPipelineSelectorCurrentIndexChanged(int index)
{
  if( current_sequence_ && !updatingControls_ ) {

    c_image_processing_pipeline::sptr current_pipeline;

    const std::string pipeline_name =
        pipelineSelector_ctl->currentText().toStdString();

    if( !pipeline_name.empty() ) {
      current_sequence_->set_current_pipeline(pipeline_name);
    }

    if( !(current_pipeline = current_sequence_->current_pipeline()) ) {
      updateCurrentSettingsWidget(nullptr);
    }
    else {
      updateCurrentSettingsWidget(current_pipeline);
    }

    Q_EMIT parameterChanged();
  }
}

void QPipelineOptionsView::onMenuButtonClicked()
{
  if( current_sequence_ ) {

    if ( menu_.isEmpty() ) {
      menu_.addAction("Add pipeline ...", this, &ThisClass::onAddPipeline);
      menu_.addAction("Rename pipeline ...", this, &ThisClass::onRenamePipeline);
      menu_.addAction("Remove pipeline ...", this, &ThisClass::onRemovePipeline);
    }

    menu_.exec(menuButton->mapToGlobal(QPoint(
        menuButton->width() / 2,
        menuButton->height() / 2)));
  }
}

void QPipelineOptionsView::onAddPipeline()
{
  if( current_sequence_ ) {

    QAddPipelineDialogBox dlgbox(this);

    if( dlgbox.exec() == QDialog::Accepted ) {

      std::string name =
          dlgbox.selectedPipelineName().toStdString();

      const std::string pipeline_class =
          dlgbox.selectedPipelineClass().toStdString();

      if( !pipeline_class.empty() ) {

        if( name.empty() ) {
          for( int i = 1; i < 10000; ++i ) {
            name = ssprintf("%s%d", pipeline_class.c_str(), i);
            if( !current_sequence_->pipeline_exists(name) ) {
              break;
            }
          }
        }
        else if( current_sequence_->pipeline_exists(name) ) {
          for( int i = 1; i < 10000; ++i ) {
            std::string newname = ssprintf("%s%d", name.c_str(), i);
            if( !current_sequence_->pipeline_exists(newname) ) {
              name = newname;
              break;
            }
          }
        }

        c_image_processing_pipeline::sptr pipeline =
            c_image_processing_pipeline::create_instance(pipeline_class, name,
                current_sequence_->input_sequence());

        if( pipeline ) {
          pipeline->set_sequence_name(current_sequence_->name());
          current_sequence_->set_current_pipeline(pipeline);
          oncurrentpipelinechanged();
        }
        else {
          QMessageBox::critical(this, "ERROR",
              qsprintf("c_image_processing_pipeline::create_instance(%s) fails",
                  pipeline_class.c_str()));
        }
      }
    }
  }

}

void QPipelineOptionsView::onRemovePipeline()
{
  if( current_sequence_ ) {

  }

}

void QPipelineOptionsView::onRenamePipeline()
{
  if( current_sequence_ ) {

  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
QAddPipelineDialogBox::QAddPipelineDialogBox(QWidget * parent) :
    Base(parent)
{
  form_ = new QFormLayout(this);

  form_->addRow("Name:", pipelineName_ctl = new QLineEditBox(this));
  form_->addRow("Type:", pipelineTypeSelector_ctl = new QComboBox(this));
  form_->addRow(pipelineTooltop_ctl = new QLabel(this));
  pipelineTooltop_ctl->setTextFormat(Qt::RichText);

  form_->addRow(hbox_ = new QHBoxLayout());
  hbox_->addWidget(btnOk_ = new QPushButton("OK"));
  hbox_->addWidget(btnCancel_ = new QPushButton("Cancel"));

  for( const auto &item : c_image_processing_pipeline::registered_classes() ) {
    pipelineTypeSelector_ctl->addItem(item.class_name.c_str(),
        QVariant::fromValue(QString(item.tooltip.c_str())));
  }

  pipelineTooltop_ctl->setTextFormat(Qt::RichText);
  pipelineTooltop_ctl->setText(pipelineTypeSelector_ctl->currentData().toString());

  connect(btnOk_, &QPushButton::clicked,
      [this]() {
        if ( pipelineTypeSelector_ctl->currentText().isEmpty() ) {
          pipelineTypeSelector_ctl->setFocus();
        }
        else {
          Base::accept();
        }
      });

  connect(btnCancel_, &QPushButton::clicked,
      [this]() {
        Base::reject();
      });

  connect(pipelineTypeSelector_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      [this](int index) {
        pipelineTooltop_ctl->setText(pipelineTypeSelector_ctl->currentData().toString());
      });

}

QString QAddPipelineDialogBox::selectedPipelineName() const
{
  return pipelineName_ctl->text();
}

QString QAddPipelineDialogBox::selectedPipelineClass() const
{
  return pipelineTypeSelector_ctl->currentText();
}


///////////////////////////////////////////////////////////////////////////////////////////////////
