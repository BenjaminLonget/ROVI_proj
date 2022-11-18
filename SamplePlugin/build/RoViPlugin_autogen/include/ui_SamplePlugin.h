/********************************************************************************
** Form generated from reading UI file 'SamplePlugin.ui'
**
** Created by: Qt User Interface Compiler version 6.2.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SAMPLEPLUGIN_H
#define UI_SAMPLEPLUGIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SamplePlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QPushButton *_btn_im;
    QPushButton *_btn_scan;
    QPushButton *_btn0;
    QPushButton *_btn1;
    QSpinBox *_spinBox;
    QSlider *_slider;
    QLabel *_label;

    void setupUi(QDockWidget *SamplePlugin)
    {
        if (SamplePlugin->objectName().isEmpty())
            SamplePlugin->setObjectName(QString::fromUtf8("SamplePlugin"));
        SamplePlugin->resize(476, 479);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        _btn_im = new QPushButton(dockWidgetContents);
        _btn_im->setObjectName(QString::fromUtf8("_btn_im"));

        verticalLayout->addWidget(_btn_im);

        _btn_scan = new QPushButton(dockWidgetContents);
        _btn_scan->setObjectName(QString::fromUtf8("_btn_scan"));

        verticalLayout->addWidget(_btn_scan);

        _btn0 = new QPushButton(dockWidgetContents);
        _btn0->setObjectName(QString::fromUtf8("_btn0"));

        verticalLayout->addWidget(_btn0);

        _btn1 = new QPushButton(dockWidgetContents);
        _btn1->setObjectName(QString::fromUtf8("_btn1"));

        verticalLayout->addWidget(_btn1);

        _spinBox = new QSpinBox(dockWidgetContents);
        _spinBox->setObjectName(QString::fromUtf8("_spinBox"));

        verticalLayout->addWidget(_spinBox);

        _slider = new QSlider(dockWidgetContents);
        _slider->setObjectName(QString::fromUtf8("_slider"));
        _slider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(_slider);

        _label = new QLabel(dockWidgetContents);
        _label->setObjectName(QString::fromUtf8("_label"));

        verticalLayout->addWidget(_label);


        verticalLayout_2->addLayout(verticalLayout);

        SamplePlugin->setWidget(dockWidgetContents);

        retranslateUi(SamplePlugin);

        QMetaObject::connectSlotsByName(SamplePlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SamplePlugin)
    {
        SamplePlugin->setWindowTitle(QCoreApplication::translate("SamplePlugin", "DockWidget", nullptr));
        _btn_im->setText(QCoreApplication::translate("SamplePlugin", "Get Image", nullptr));
        _btn_scan->setText(QCoreApplication::translate("SamplePlugin", "Get Scan", nullptr));
        _btn0->setText(QCoreApplication::translate("SamplePlugin", "Calculate Path", nullptr));
        _btn1->setText(QCoreApplication::translate("SamplePlugin", "Run Path", nullptr));
        _label->setText(QCoreApplication::translate("SamplePlugin", "Label", nullptr));
    } // retranslateUi

};

namespace Ui {
    class SamplePlugin: public Ui_SamplePlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SAMPLEPLUGIN_H
