/*
 * QImageProcessorSelector.cc
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#include "QImageProcessorSelector.h"
#include "QImageProcessorsCollection.h"
#include <gui/widgets/style.h>
#include <core/debug.h>


#define ICON_double_arrow_down    ":/qimproc/icons/double-arrow-down"
#define ICON_double_arrow_right   ":/qimproc/icons/double-arrow-right"
#define ICON_move_down            ":/qimproc/icons/move-down"
#define ICON_move_up              ":/qimproc/icons/move-up"
#define ICON_delete               ":/qimproc/icons/delete"
#define ICON_add                  ":/qimproc/icons/add"
#define ICON_rename               ":/qimproc/icons/rename"
#define ICON_copy                 ":/qimproc/icons/copy"
#define ICON_menu                 ":/qimproc/icons/menu"


//static QIcon getIcon(const QString & name)
//{
//  return QIcon(QString(":/qimproc/icons/%1").arg(name));
//}
//

QImageProcessorSelector::QImageProcessorSelector(QWidget * parent)
  : Base("QImageProcessorSelector", parent)
{

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



  Q_INIT_RESOURCE(qimproc_resources);

  if ( QImageProcessorsCollection::empty() ) {
    QImageProcessorsCollection::load();
  }

  enabled_ctl = add_checkbox("Enabled",
      [this](bool /*checked*/) {
        //emit imageProcessingEnableChanged(state == Qt::Checked);
        Q_EMIT parameterChanged();
      });


  selectorToolbar = new QToolBar(this);
  selectorToolbar->setToolButtonStyle(Qt::ToolButtonStyle::ToolButtonIconOnly);
  selectorToolbar->setIconSize(QSize(16, 16));

  selector_ctl = new QComboBox(this);
  selector_ctl->setEditable(false);
  selector_ctl->setMinimumContentsLength(12);
  selector_ctl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  selectorToolbar->addWidget(selector_ctl);

  selectorMenu_ctl = new QToolButton();
  selectorMenu_ctl->setIcon(getIcon(ICON_menu));
  selectorMenu_ctl->setToolTip("Popup menu");
  selectorToolbar->addWidget(selectorMenu_ctl);

  form->addRow(selectorToolbar);

  chain_ctl = new QImageProcessorChainEditor(this);
  form->addRow(createScrollableWrap(chain_ctl, this));


  connect(selectorMenu_ctl, &QToolButton::clicked,
      [this] () {

        QMenu menu;
        QAction * add_processor_action = Q_NULLPTR;
        QAction * rename_processor_action = Q_NULLPTR;
        QAction * copy_processor_action = Q_NULLPTR;
        QAction * delete_processor_action = Q_NULLPTR;

        menu.addAction(add_processor_action =
            new QAction(getIcon(ICON_add),
                "Add ..."));

        if ( current_processor_ ) {

          // Depp copy is not implemented yet
          //  menu.addAction(copy_processor_action =
          //    new QAction(getIcon(ICON_copy),
          //      "Add copy ..."));

          menu.addAction(rename_processor_action =
              new QAction(getIcon(ICON_rename),
                  "Rename ..."));

          menu.addAction(delete_processor_action =
              new QAction(getIcon(ICON_delete),
                  "Delete ..."));
        }


        QAction * selectedAction =
            menu.exec(selectorMenu_ctl->
                mapToGlobal(QPoint(selectorMenu_ctl->width()-4,
                    selectorMenu_ctl->height()-4)));

        if ( selectedAction ) {
          if ( selectedAction == add_processor_action ) {
            addProcessor();
          }
          else if ( selectedAction == copy_processor_action ) {
            addCopyOfCurrentProcessor();
          }
          else if ( selectedAction == rename_processor_action ) {
            renameCurrentProcessor();
          }
          else if ( selectedAction == delete_processor_action ) {
            deleteCurrentProcessor();
          }
        }

      });


  connect(selector_ctl, SIGNAL(currentIndexChanged(int)),
      this, SLOT(onProcessorSelectorCurrentIndexChanged(int)) );

  connect(chain_ctl, &QImageProcessorChainEditor::parameterChanged,
      [this]() {
        if ( enabled_ctl->isChecked() ) {
          Q_EMIT parameterChanged();
        }
      });

  updateControls();
}

//void QImageProcessorSelector::set_available_processors(const c_image_processor_collection::ptr & processors)
//{
//  available_processors_ = processors;
//  updateControls();
//}
//
//const c_image_processor_collection::ptr QImageProcessorSelector::available_processors() const
//{
//  return available_processors_;
//}

c_image_processor::ptr QImageProcessorSelector::current_processor() const
{
  return enabled_ctl->isChecked() ? current_processor_ : nullptr;
}

bool QImageProcessorSelector::imageProcessingEnabled() const
{
  return enabled_ctl->isChecked();
}

void QImageProcessorSelector::onupdatecontrols()
{
  selector_ctl->clear();

  for ( size_t i = 0, n = QImageProcessorsCollection::size(); i < n; ++i ) {
    const c_image_processor::ptr & processor = QImageProcessorsCollection::item(i);
    if ( processor ) {
      selector_ctl->addItem(processor->cname(), QVariant((int) (i)));
      selector_ctl->setItemData(i, processor->cfilename(), Qt::ToolTipRole);
    }
  }

  if ( selector_ctl->count() > 0 ) {

    if ( !current_processor_ ) {
      selector_ctl->setCurrentIndex(0);
    }
    else {
      const int index = selector_ctl->findText(current_processor_->cname());
      selector_ctl->setCurrentIndex(index >= 0 ? index : 0);
    }
  }

  setEnabled(true);

  updatecurrentprocessor();

  Base::onupdatecontrols();
}

void QImageProcessorSelector::onProcessorSelectorCurrentIndexChanged(int)
{
  if ( !updatingControls() ) {
    updatecurrentprocessor();
  }
}

void QImageProcessorSelector::updatecurrentprocessor()
{
  c_image_processor::ptr selected_processor;

  if( !QImageProcessorsCollection::empty() ) {

    QString processorName = selector_ctl->currentText();
    if ( !processorName.isEmpty() ) {
      const int pos = QImageProcessorsCollection::indexof(processorName);
      if ( pos >= 0 ) {
        selected_processor = QImageProcessorsCollection::item(pos);
      }
    }
  }

  if ( current_processor_ != selected_processor  ) {
    current_processor_ = selected_processor;
  }

  if ( current_processor_ != chain_ctl->current_processor() ) {
    chain_ctl->set_current_processor(current_processor_);
  }
}

void QImageProcessorSelector::addProcessor()
{
  while ( 42 ) {

    const QString processorName =
        QInputDialog::getText(this,
            "Add image processor...",
            "Processor name:",
            QLineEdit::EchoMode::Normal);

    if ( processorName.isEmpty() ) {
      return;
    }


    if ( QImageProcessorsCollection::indexof(processorName) >=0 ) {
      QMessageBox::warning(this, "ERROR", "Processor with this name is already exists.\n"
          "Enter another name.");
      continue;
    }

    c_image_processor::ptr processor =
        c_image_processor::create(processorName.toStdString());

    if ( !processor ) {
      QMessageBox::critical(this, "FATAL ERROR", "Unexpected error:\n"
          "c_image_processor::create() fails.");
      return;
    }

    QImageProcessorsCollection::add(current_processor_ = processor);
    current_processor_->save();
    updateControls();

    return;
  }

}

void QImageProcessorSelector::deleteCurrentProcessor()
{
  if ( !current_processor_ ) {
    return;
  }

  const int reply = QMessageBox::question(this,
      "Confirmation required",
      QString("Are you sure to deletec current processor '%1' ?").
          arg(current_processor_->cname()),
      QMessageBox::Yes | QMessageBox::No);


  if ( reply != QMessageBox::Yes ) {
    return;
  }

  const int pos = QImageProcessorsCollection::indexof(current_processor_);
  if( pos < 0 ) {
    return;
  }

  QImageProcessorsCollection::remove_at(pos);

  if ( !current_processor_->filename().empty() ) {

    if ( QFile::exists(current_processor_->cfilename()) ) {

      if ( !QFile::remove(current_processor_->cfilename()) ) {

        QMessageBox::warning(this, "WARNING",
            QString("Failed to remove the file from disk:\n"
                "%1\n"
                "Check the path and file protection."));
      }
    }
  }

  current_processor_.reset();
  updateControls();
}

void QImageProcessorSelector::renameCurrentProcessor()
{
  if ( !current_processor_ ) {
    return;
  }

  QString processorName = current_processor_->cname();

  while ( 42 ) {

    processorName =
        QInputDialog::getText(this,
            "Rename current processor...",
            "New processor name:",
            QLineEdit::EchoMode::Normal,
            processorName);

    if ( processorName.isEmpty() || processorName.compare(current_processor_->cname()) == 0 ) {
      return;
    }

    if ( !QImageProcessorsCollection::empty() ) {
      if ( QImageProcessorsCollection::indexof(processorName) >=0 ) {
        QMessageBox::warning(this, "ERROR", "Processor with this name is already exists.\n"
            "Enter another name.");
        continue;
      }
    }

    current_processor_->set_name(processorName.toStdString());

    if ( !current_processor_->filename().empty() ) {

      QFile::remove(current_processor_->cfilename());

      const QString abspath = QFileInfo(current_processor_->cfilename()).absolutePath();
      if ( !abspath.isEmpty() ) {
        current_processor_->set_filename(QString("%1/%2.cfg").arg(abspath).arg(processorName).toStdString());
      }
    }

    current_processor_->save();

    updateControls();

    return;
  }

}

void QImageProcessorSelector::addCopyOfCurrentProcessor()
{
  // not implemented yet
}


///////////////////////////////////////////////////////////////////////////////////////////////////

