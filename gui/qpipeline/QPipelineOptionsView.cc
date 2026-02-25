/*
 * QPipelineOptionsView.cc
 *
 *  Created on: Feb 22, 2023
 *      Author: amyznikov
 */

#include "QPipelineOptionsView.h"
#include "QPipelineThread.h"
#include <gui/widgets/qsprintf.h>
#include <gui/widgets/style.h>
#include <core/ssprintf.h>

#define ICON_close          ":/qpipeline/icons/close"
#define ICON_menu           ":/qpipeline/icons/menu"
#define ICON_checkall       ":/qpipeline/icons/check_all"

QPipelineOptionsView::QPipelineOptionsView(QWidget * parent) :
  Base(parent)
{
  Q_INIT_RESOURCE(qpipeline_resources);

  static const auto addSpacer =
      [](QToolBar * toolBar) -> QWidget *
  {
    QWidget * spacer = new QWidget();
    spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    toolBar->addWidget(spacer);
    return spacer;
  };


  _layout = new QVBoxLayout(this);

  _toolbar = new QToolBar();
  _toolbar->setIconSize(QSize(16, 16));

  _toolbar->addWidget(sequenceName_lb = new QLabel(""));
  sequenceName_lb->setTextFormat(Qt::RichText);
  _toolbar->addSeparator();

  _toolbar->addWidget(pipelineSelector_ctl = new QComboBox(this));
  pipelineSelector_ctl->setEditable(false);
  pipelineSelector_ctl->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  pipelineSelector_ctl->setToolTip("Select current image processing pipeline");
  connect(pipelineSelector_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      this, &ThisClass::onPipelineSelectorCurrentIndexChanged);


  _toolbar->addWidget(menuButton_ctl = new QToolButton());
  menuButton_ctl->setText("Options...");
  menuButton_ctl->setIcon(getIcon(ICON_menu));
  menuButton_ctl->setToolTip("Add / Remove image processing pipelines");
  menuButton_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  connect(menuButton_ctl, &QToolButton::clicked,
      this, &ThisClass::onMenuButtonClicked);

  _toolbar->addWidget(cloneButton_ctl = new QToolButton());
  cloneButton_ctl->setText("Clone");
  cloneButton_ctl->setIcon(getIcon(ICON_checkall));
  cloneButton_ctl->setToolTip("Clone current pipeline / options into all selected image sequences");
  cloneButton_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  connect(cloneButton_ctl, &QToolButton::clicked,
      [this]() {
        if( _current_sequence && _current_sequence->current_pipeline() ) {
          Q_EMIT cloneCurrentPipelineRequested();
        }
      });


  _toolbar->addSeparator();

  addSpacer(_toolbar);

  _toolbar->addAction(close_ctl = new QAction(getIcon(ICON_close), "Close"));
  close_ctl->setShortcut(QKeySequence::Cancel);
  close_ctl->setToolTip("Close window");
  connect(close_ctl, &QAction::triggered, this,
      &ThisClass::closeWindowRequested);


  scrollArea_ctl = new QScrollArea(this);
  scrollArea_ctl->setWidgetResizable(true);
  scrollArea_ctl->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
  scrollArea_ctl->setFrameShape(QFrame::NoFrame);


  _layout->addWidget(_toolbar, 1, Qt::AlignTop);
  _layout->addWidget(scrollArea_ctl, 1000);
}

void QPipelineOptionsView::set_current_sequence(const c_image_sequence::sptr & image_sequence)
{
  _updatingControls = true;
  pipelineSelector_ctl->clear();

  if( !(_current_sequence = image_sequence) ) {
    enablecontrols(false);
    sequenceName_lb->setText("");
    updateCurrentSettingsWidget(nullptr);
  }
  else {

    sequenceName_lb->setText(qsprintf("<strong>%s</strong>",
        _current_sequence->name().c_str()));

    const std::vector<c_image_processing_pipeline::sptr> &pipelines =
        _current_sequence->pipelines();

    for( const c_image_processing_pipeline::sptr &pipeline : pipelines ) {
      pipelineSelector_ctl->addItem(pipeline->name().c_str());
    }

    const c_image_processing_pipeline::sptr &current_pipeline =
        _current_sequence->current_pipeline();

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

  _updatingControls = false;
}

const c_image_sequence::sptr & QPipelineOptionsView::current_sequence() const
{
  return _current_sequence;
}

void QPipelineOptionsView::updateCurrentSettingsWidget(const c_image_processing_pipeline::sptr &pipeline)
{
  QPipelineSettingsWidget *currentWidget = nullptr;

  QImageProcessingPipeline * pp = pipeline ?
      dynamic_cast<QImageProcessingPipeline * >(pipeline.get()) :
      nullptr;

  if( pp ) {

    const QString className =
        pp->className();

    for( int i = 0, n = _pipelineSettingsWidgets.size(); i < n; ++i ) {
      if( className == _pipelineSettingsWidgets[i]->pipelineClass() ) {
        currentWidget = _pipelineSettingsWidgets[i];
        break;
      }
    }

    if( !currentWidget ) {

      _pipelineSettingsWidgets.append(currentWidget =
          pp->createSettingsWidget(this));

      connect(currentWidget, &QSettingsWidget::parameterChanged,
          this, &ThisClass::parameterChanged);

    }
  }

  QPipelineSettingsWidget * oldWidget =
      dynamic_cast<QPipelineSettingsWidget*>(scrollArea_ctl->widget());

  if( oldWidget != currentWidget ) {

    if( oldWidget ) {
      oldWidget->setCurrentPipeline(nullptr);
      oldWidget->hide();
    }

    scrollArea_ctl->takeWidget();
    scrollArea_ctl->setWidget(currentWidget);
  }

  if( currentWidget ) {
    currentWidget->setCurrentPipeline(pp);
    currentWidget->setEnabled(!QPipelineThread::isRunning() || pipeline != QPipelineThread::currentPipeline());
    currentWidget->show();
  }

}

void QPipelineOptionsView::enablecontrols(bool v)
{
  menuButton_ctl->setEnabled(v);
  cloneButton_ctl->setEnabled(v && _current_sequence && _current_sequence->current_pipeline());
  pipelineSelector_ctl->setEnabled(v && pipelineSelector_ctl->count() > 0);
}

void QPipelineOptionsView::oncurrentpipelinechanged()
{
  if( _current_sequence ) {

    _updatingControls = true;

    const c_image_processing_pipeline::sptr &current_pipeline =
        _current_sequence->current_pipeline();

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

    _updatingControls = false;

    Q_EMIT parameterChanged();
  }
}


void QPipelineOptionsView::onPipelineSelectorCurrentIndexChanged(int index)
{
  if( _current_sequence && !_updatingControls ) {

    c_image_processing_pipeline::sptr current_pipeline;

    const std::string pipeline_name =
        pipelineSelector_ctl->currentText().toStdString();

    if( !pipeline_name.empty() ) {
      _current_sequence->set_current_pipeline(pipeline_name);
    }

    if( !(current_pipeline = _current_sequence->current_pipeline()) ) {
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
  if( _current_sequence ) {

    if ( _menu.isEmpty() ) {
      _menu.addAction("Add pipeline ...", this, &ThisClass::onAddPipeline);
      _menu.addAction("Rename pipeline ...", this, &ThisClass::onRenamePipeline);
      _menu.addAction("Remove pipeline ...", this, &ThisClass::onRemovePipeline);
    }

    _menu.exec(menuButton_ctl->mapToGlobal(QPoint(
        menuButton_ctl->width() / 2,
        menuButton_ctl->height() / 2)));
  }
}

void QPipelineOptionsView::onAddPipeline()
{
  if( _current_sequence ) {

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
            if( !_current_sequence->pipeline_exists(name) ) {
              break;
            }
          }
        }
        else if( _current_sequence->pipeline_exists(name) ) {
          for( int i = 1; i < 10000; ++i ) {
            std::string newname = ssprintf("%s%d", name.c_str(), i);
            if( !_current_sequence->pipeline_exists(newname) ) {
              name = newname;
              break;
            }
          }
        }

        c_image_processing_pipeline::sptr pipeline =
            c_image_processing_pipeline::create_instance(pipeline_class, name,
                _current_sequence);

        if( pipeline ) {
          //pipeline->set_sequence_name(current_sequence_->name());
          _current_sequence->set_current_pipeline(pipeline);
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
  if( _current_sequence && _current_sequence->current_pipeline() ) {

    const c_image_processing_pipeline::sptr pipeline =
        _current_sequence->current_pipeline();

    const int responce =
        QMessageBox::warning(this,
            "Remove Pipeline",
            qsprintf("Are you sure to remove pipeline %s? ",
                pipeline->name().c_str()),
            QMessageBox::Yes | QMessageBox::No);

    if ( responce != QMessageBox::Yes ) {
      return;
    }


    _current_sequence->remove_pipeline(pipeline);

    const int index =
        pipelineSelector_ctl->findText(pipeline->name().c_str());

    if ( index >= 0 ) {
      pipelineSelector_ctl->removeItem(index);
    }

    Q_EMIT parameterChanged();
  }
}

void QPipelineOptionsView::onRenamePipeline()
{
  if( _current_sequence ) {

    const c_image_processing_pipeline::sptr &pipeline =
        _current_sequence->current_pipeline();

    if ( pipeline ) {

      while ( 42 ) {

        const std::string current_name =
            pipeline->name();

        const std::string new_name =
            QInputDialog::getText(this,
                "Rename Pipeline",
                "New name:",
                QLineEdit::Normal,
                current_name.c_str()).toStdString();

        if( new_name.empty() ) {
          break;
        }

        if( !_current_sequence->find_pipeline(new_name) ) {

          pipeline->set_name(new_name);

          const int index = pipelineSelector_ctl->findText(current_name.c_str());
          if( index >= 0 ) {
            pipelineSelector_ctl->setItemText(index, new_name.c_str());
          }

          Q_EMIT parameterChanged();
          break;
        }

        QMessageBox::warning(this,
            "Rename Pipeline",
            "Error: Pipeline '%s' already exists for this sequence.\n"
                "Enter another name.");
      }
    }

  }
}


bool QPipelineOptionsView::cloneCurrentPipeline(const std::vector<c_image_sequence::sptr> & sequences)
{
  if( !_current_sequence ) {
    return false;
  }

  const c_image_processing_pipeline::sptr &current_pipeline =
      _current_sequence->current_pipeline();
  if( !current_pipeline ) {
    return false;
  }

  bool has_changes = false;

  for ( const c_image_sequence::sptr & sequence : sequences ) {

    c_image_processing_pipeline::sptr pipeline =
        sequence->find_pipeline(current_pipeline->name());

    if( !pipeline ) {

      pipeline =
          c_image_processing_pipeline::create_instance(current_pipeline->get_class_name(),
              current_pipeline->name(),
              sequence);

      if( pipeline ) {
        sequence->add_pipeline(pipeline);
      }
      else {
        CF_ERROR("c_image_processing_pipeline::create_instance() fails");
        continue;
      }
    }

    if( pipeline != current_pipeline ) {
      if( !current_pipeline->copyParameters(pipeline) ) {
        CF_ERROR("current_pipeline->copyParameters() fails for pipeline '%s'",
            pipeline->get_class_name().c_str());
        continue;
      }

      has_changes = true;
    }
  }


  return has_changes;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
QAddPipelineDialogBox::QAddPipelineDialogBox(QWidget * parent) :
    Base(parent)
{
  _form = new QFormLayout(this);

  _form->addRow("Name:", pipelineName_ctl = new QLineEditBox(this));
  _form->addRow("Type:", pipelineTypeSelector_ctl = new QComboBox(this));
  _form->addRow(pipelineTooltop_ctl = new QLabel(this));
  pipelineTooltop_ctl->setTextFormat(Qt::RichText);

  _form->addRow(_hbox = new QHBoxLayout());
  _hbox->addWidget(btnOk_ctl = new QPushButton("OK"));
  _hbox->addWidget(btnCancel_ctl = new QPushButton("Cancel"));

  for( const auto &item : c_image_processing_pipeline::registered_classes() ) {
    pipelineTypeSelector_ctl->addItem(item.class_name.c_str(),
        QVariant::fromValue(QString(item.tooltip.c_str())));
  }

  pipelineTooltop_ctl->setTextFormat(Qt::RichText);
  pipelineTooltop_ctl->setText(pipelineTypeSelector_ctl->currentData().toString());

  connect(btnOk_ctl, &QPushButton::clicked,
      [this]() {
        if ( pipelineTypeSelector_ctl->currentText().isEmpty() ) {
          pipelineTypeSelector_ctl->setFocus();
        }
        else {
          Base::accept();
        }
      });

  connect(btnCancel_ctl, &QPushButton::clicked,
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
