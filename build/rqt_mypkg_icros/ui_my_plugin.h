/********************************************************************************
** Form generated from reading UI file 'my_plugin.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MY_PLUGIN_H
#define UI_MY_PLUGIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MyPluginWidget
{
public:
    QHBoxLayout *horizontalLayout_4;
    QWidget *widget;
    QWidget *widget_2;
    QWidget *gridLayoutWidget;
    QGridLayout *Cmd_;
    QLabel *cmd_x;
    QLabel *cmd_y;
    QLabel *cmd_z;
    QWidget *gridLayoutWidget_2;
    QGridLayout *cmd_QScroll;
    QScrollBar *qsc_y;
    QScrollBar *qsc_x;
    QScrollBar *qsc_z;
    QGroupBox *GroupBox_Measured;
    QLabel *FK_x;
    QLineEdit *txt_joint1;
    QLabel *rad3;
    QLineEdit *txt_joint2;
    QLabel *FK_ym;
    QLabel *rad2;
    QLabel *label_24;
    QLabel *rad4;
    QLineEdit *txt_joint4;
    QLabel *Joint1;
    QLabel *rad1;
    QLineEdit *txt_joint3;
    QLabel *label_26;
    QLabel *FK_xm;
    QLabel *Joint2;
    QLineEdit *txt_FK_z;
    QLabel *Joint4;
    QLabel *FK_zm;
    QLineEdit *txt_FK_y;
    QLabel *FK_z;
    QLineEdit *txt_FK_x;
    QLineEdit *txt_Error_y;
    QLabel *label_30;
    QLabel *label_9;
    QLineEdit *txt_Error_x;
    QLabel *label_29;
    QLabel *label_8;
    QLineEdit *txt_Error_z;
    QLabel *label_28;
    QLabel *lbl_x;
    QLabel *label_27;
    QLabel *label_31;
    QPushButton *btn_Start;
    QWidget *gridLayoutWidget_3;
    QGridLayout *cmd_value;
    QLabel *lbl_cmd_x;
    QLabel *lbl_cmd_y;
    QLabel *lbl_cmd_z;
    QCheckBox *chk_Publish;
    QPlainTextEdit *pte_stateLog;
    QLabel *lbl_ping_update;
    QLabel *lbl_ping;

    void setupUi(QWidget *MyPluginWidget)
    {
        if (MyPluginWidget->objectName().isEmpty())
            MyPluginWidget->setObjectName(QStringLiteral("MyPluginWidget"));
        MyPluginWidget->resize(701, 849);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MyPluginWidget->sizePolicy().hasHeightForWidth());
        MyPluginWidget->setSizePolicy(sizePolicy);
        horizontalLayout_4 = new QHBoxLayout(MyPluginWidget);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        widget = new QWidget(MyPluginWidget);
        widget->setObjectName(QStringLiteral("widget"));
        widget_2 = new QWidget(widget);
        widget_2->setObjectName(QStringLiteral("widget_2"));
        widget_2->setGeometry(QRect(0, 30, 671, 491));
        gridLayoutWidget = new QWidget(widget_2);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(0, 10, 89, 151));
        Cmd_ = new QGridLayout(gridLayoutWidget);
        Cmd_->setObjectName(QStringLiteral("Cmd_"));
        Cmd_->setContentsMargins(0, 0, 0, 0);
        cmd_x = new QLabel(gridLayoutWidget);
        cmd_x->setObjectName(QStringLiteral("cmd_x"));

        Cmd_->addWidget(cmd_x, 0, 0, 1, 1);

        cmd_y = new QLabel(gridLayoutWidget);
        cmd_y->setObjectName(QStringLiteral("cmd_y"));

        Cmd_->addWidget(cmd_y, 1, 0, 1, 1);

        cmd_z = new QLabel(gridLayoutWidget);
        cmd_z->setObjectName(QStringLiteral("cmd_z"));

        Cmd_->addWidget(cmd_z, 2, 0, 1, 1);

        gridLayoutWidget_2 = new QWidget(widget_2);
        gridLayoutWidget_2->setObjectName(QStringLiteral("gridLayoutWidget_2"));
        gridLayoutWidget_2->setGeometry(QRect(80, 10, 501, 151));
        cmd_QScroll = new QGridLayout(gridLayoutWidget_2);
        cmd_QScroll->setObjectName(QStringLiteral("cmd_QScroll"));
        cmd_QScroll->setContentsMargins(0, 0, 0, 0);
        qsc_y = new QScrollBar(gridLayoutWidget_2);
        qsc_y->setObjectName(QStringLiteral("qsc_y"));
        qsc_y->setMaximum(10000000);
        qsc_y->setSingleStep(1);
        qsc_y->setValue(0);
        qsc_y->setSliderPosition(0);
        qsc_y->setOrientation(Qt::Horizontal);

        cmd_QScroll->addWidget(qsc_y, 1, 0, 1, 1);

        qsc_x = new QScrollBar(gridLayoutWidget_2);
        qsc_x->setObjectName(QStringLiteral("qsc_x"));
        qsc_x->setMinimum(0);
        qsc_x->setMaximum(10000000);
        qsc_x->setValue(6000000);
        qsc_x->setSliderPosition(6000000);
        qsc_x->setOrientation(Qt::Horizontal);

        cmd_QScroll->addWidget(qsc_x, 0, 0, 1, 1);

        qsc_z = new QScrollBar(gridLayoutWidget_2);
        qsc_z->setObjectName(QStringLiteral("qsc_z"));
        qsc_z->setMinimum(0);
        qsc_z->setMaximum(10000000);
        qsc_z->setValue(0);
        qsc_z->setOrientation(Qt::Horizontal);

        cmd_QScroll->addWidget(qsc_z, 2, 0, 1, 1);

        GroupBox_Measured = new QGroupBox(widget_2);
        GroupBox_Measured->setObjectName(QStringLiteral("GroupBox_Measured"));
        GroupBox_Measured->setGeometry(QRect(0, 180, 671, 191));
        FK_x = new QLabel(GroupBox_Measured);
        FK_x->setObjectName(QStringLiteral("FK_x"));
        FK_x->setGeometry(QRect(210, 80, 59, 14));
        txt_joint1 = new QLineEdit(GroupBox_Measured);
        txt_joint1->setObjectName(QStringLiteral("txt_joint1"));
        txt_joint1->setEnabled(true);
        txt_joint1->setGeometry(QRect(60, 50, 81, 22));
        txt_joint1->setLayoutDirection(Qt::LeftToRight);
        txt_joint1->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        txt_joint1->setReadOnly(true);
        rad3 = new QLabel(GroupBox_Measured);
        rad3->setObjectName(QStringLiteral("rad3"));
        rad3->setGeometry(QRect(145, 114, 59, 14));
        txt_joint2 = new QLineEdit(GroupBox_Measured);
        txt_joint2->setObjectName(QStringLiteral("txt_joint2"));
        txt_joint2->setEnabled(true);
        txt_joint2->setGeometry(QRect(60, 80, 81, 22));
        txt_joint2->setLayoutDirection(Qt::LeftToRight);
        txt_joint2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        txt_joint2->setReadOnly(true);
        FK_ym = new QLabel(GroupBox_Measured);
        FK_ym->setObjectName(QStringLiteral("FK_ym"));
        FK_ym->setGeometry(QRect(315, 110, 59, 14));
        rad2 = new QLabel(GroupBox_Measured);
        rad2->setObjectName(QStringLiteral("rad2"));
        rad2->setGeometry(QRect(145, 84, 59, 14));
        label_24 = new QLabel(GroupBox_Measured);
        label_24->setObjectName(QStringLiteral("label_24"));
        label_24->setGeometry(QRect(10, 114, 59, 14));
        rad4 = new QLabel(GroupBox_Measured);
        rad4->setObjectName(QStringLiteral("rad4"));
        rad4->setGeometry(QRect(145, 144, 59, 14));
        txt_joint4 = new QLineEdit(GroupBox_Measured);
        txt_joint4->setObjectName(QStringLiteral("txt_joint4"));
        txt_joint4->setEnabled(true);
        txt_joint4->setGeometry(QRect(60, 140, 81, 22));
        txt_joint4->setLayoutDirection(Qt::LeftToRight);
        txt_joint4->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        txt_joint4->setReadOnly(true);
        Joint1 = new QLabel(GroupBox_Measured);
        Joint1->setObjectName(QStringLiteral("Joint1"));
        Joint1->setGeometry(QRect(10, 54, 59, 14));
        rad1 = new QLabel(GroupBox_Measured);
        rad1->setObjectName(QStringLiteral("rad1"));
        rad1->setGeometry(QRect(145, 54, 59, 14));
        txt_joint3 = new QLineEdit(GroupBox_Measured);
        txt_joint3->setObjectName(QStringLiteral("txt_joint3"));
        txt_joint3->setEnabled(true);
        txt_joint3->setGeometry(QRect(60, 110, 81, 22));
        txt_joint3->setLayoutDirection(Qt::LeftToRight);
        txt_joint3->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        txt_joint3->setReadOnly(true);
        label_26 = new QLabel(GroupBox_Measured);
        label_26->setObjectName(QStringLiteral("label_26"));
        label_26->setGeometry(QRect(210, 110, 59, 14));
        FK_xm = new QLabel(GroupBox_Measured);
        FK_xm->setObjectName(QStringLiteral("FK_xm"));
        FK_xm->setGeometry(QRect(315, 80, 59, 14));
        Joint2 = new QLabel(GroupBox_Measured);
        Joint2->setObjectName(QStringLiteral("Joint2"));
        Joint2->setGeometry(QRect(10, 84, 59, 14));
        txt_FK_z = new QLineEdit(GroupBox_Measured);
        txt_FK_z->setObjectName(QStringLiteral("txt_FK_z"));
        txt_FK_z->setEnabled(true);
        txt_FK_z->setGeometry(QRect(230, 136, 81, 22));
        txt_FK_z->setLayoutDirection(Qt::LeftToRight);
        txt_FK_z->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        txt_FK_z->setReadOnly(true);
        Joint4 = new QLabel(GroupBox_Measured);
        Joint4->setObjectName(QStringLiteral("Joint4"));
        Joint4->setGeometry(QRect(10, 144, 59, 14));
        FK_zm = new QLabel(GroupBox_Measured);
        FK_zm->setObjectName(QStringLiteral("FK_zm"));
        FK_zm->setGeometry(QRect(315, 140, 59, 14));
        txt_FK_y = new QLineEdit(GroupBox_Measured);
        txt_FK_y->setObjectName(QStringLiteral("txt_FK_y"));
        txt_FK_y->setEnabled(true);
        txt_FK_y->setGeometry(QRect(230, 106, 81, 22));
        txt_FK_y->setLayoutDirection(Qt::LeftToRight);
        txt_FK_y->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        txt_FK_y->setReadOnly(true);
        FK_z = new QLabel(GroupBox_Measured);
        FK_z->setObjectName(QStringLiteral("FK_z"));
        FK_z->setGeometry(QRect(210, 140, 59, 14));
        txt_FK_x = new QLineEdit(GroupBox_Measured);
        txt_FK_x->setObjectName(QStringLiteral("txt_FK_x"));
        txt_FK_x->setEnabled(true);
        txt_FK_x->setGeometry(QRect(230, 76, 81, 22));
        txt_FK_x->setLayoutDirection(Qt::LeftToRight);
        txt_FK_x->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        txt_FK_x->setReadOnly(true);
        txt_Error_y = new QLineEdit(GroupBox_Measured);
        txt_Error_y->setObjectName(QStringLiteral("txt_Error_y"));
        txt_Error_y->setEnabled(true);
        txt_Error_y->setGeometry(QRect(390, 106, 81, 22));
        txt_Error_y->setLayoutDirection(Qt::LeftToRight);
        txt_Error_y->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        txt_Error_y->setReadOnly(true);
        label_30 = new QLabel(GroupBox_Measured);
        label_30->setObjectName(QStringLiteral("label_30"));
        label_30->setGeometry(QRect(475, 80, 59, 14));
        label_9 = new QLabel(GroupBox_Measured);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(370, 110, 59, 14));
        txt_Error_x = new QLineEdit(GroupBox_Measured);
        txt_Error_x->setObjectName(QStringLiteral("txt_Error_x"));
        txt_Error_x->setEnabled(true);
        txt_Error_x->setGeometry(QRect(390, 76, 81, 22));
        txt_Error_x->setLayoutDirection(Qt::LeftToRight);
        txt_Error_x->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        txt_Error_x->setReadOnly(true);
        label_29 = new QLabel(GroupBox_Measured);
        label_29->setObjectName(QStringLiteral("label_29"));
        label_29->setGeometry(QRect(475, 110, 59, 14));
        label_8 = new QLabel(GroupBox_Measured);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(370, 140, 59, 14));
        txt_Error_z = new QLineEdit(GroupBox_Measured);
        txt_Error_z->setObjectName(QStringLiteral("txt_Error_z"));
        txt_Error_z->setEnabled(true);
        txt_Error_z->setGeometry(QRect(390, 136, 81, 22));
        txt_Error_z->setLayoutDirection(Qt::LeftToRight);
        txt_Error_z->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        txt_Error_z->setReadOnly(true);
        label_28 = new QLabel(GroupBox_Measured);
        label_28->setObjectName(QStringLiteral("label_28"));
        label_28->setGeometry(QRect(475, 140, 59, 14));
        lbl_x = new QLabel(GroupBox_Measured);
        lbl_x->setObjectName(QStringLiteral("lbl_x"));
        lbl_x->setGeometry(QRect(370, 80, 59, 14));
        label_27 = new QLabel(GroupBox_Measured);
        label_27->setObjectName(QStringLiteral("label_27"));
        label_27->setGeometry(QRect(210, 50, 59, 14));
        label_31 = new QLabel(GroupBox_Measured);
        label_31->setObjectName(QStringLiteral("label_31"));
        label_31->setGeometry(QRect(370, 50, 111, 16));
        btn_Start = new QPushButton(GroupBox_Measured);
        btn_Start->setObjectName(QStringLiteral("btn_Start"));
        btn_Start->setGeometry(QRect(520, 60, 131, 101));
        btn_Start->setCheckable(true);
        gridLayoutWidget_3 = new QWidget(widget_2);
        gridLayoutWidget_3->setObjectName(QStringLiteral("gridLayoutWidget_3"));
        gridLayoutWidget_3->setGeometry(QRect(590, 10, 61, 151));
        cmd_value = new QGridLayout(gridLayoutWidget_3);
        cmd_value->setObjectName(QStringLiteral("cmd_value"));
        cmd_value->setContentsMargins(0, 0, 0, 0);
        lbl_cmd_x = new QLabel(gridLayoutWidget_3);
        lbl_cmd_x->setObjectName(QStringLiteral("lbl_cmd_x"));

        cmd_value->addWidget(lbl_cmd_x, 0, 0, 1, 1);

        lbl_cmd_y = new QLabel(gridLayoutWidget_3);
        lbl_cmd_y->setObjectName(QStringLiteral("lbl_cmd_y"));

        cmd_value->addWidget(lbl_cmd_y, 1, 0, 1, 1);

        lbl_cmd_z = new QLabel(gridLayoutWidget_3);
        lbl_cmd_z->setObjectName(QStringLiteral("lbl_cmd_z"));

        cmd_value->addWidget(lbl_cmd_z, 2, 0, 1, 1);

        chk_Publish = new QCheckBox(widget_2);
        chk_Publish->setObjectName(QStringLiteral("chk_Publish"));
        chk_Publish->setGeometry(QRect(510, 170, 151, 23));
        pte_stateLog = new QPlainTextEdit(widget_2);
        pte_stateLog->setObjectName(QStringLiteral("pte_stateLog"));
        pte_stateLog->setGeometry(QRect(10, 380, 651, 101));
        lbl_ping_update = new QLabel(widget);
        lbl_ping_update->setObjectName(QStringLiteral("lbl_ping_update"));
        lbl_ping_update->setGeometry(QRect(610, 0, 59, 46));
        lbl_ping = new QLabel(widget);
        lbl_ping->setObjectName(QStringLiteral("lbl_ping"));
        lbl_ping->setGeometry(QRect(570, -1, 59, 46));

        horizontalLayout_4->addWidget(widget);


        retranslateUi(MyPluginWidget);

        QMetaObject::connectSlotsByName(MyPluginWidget);
    } // setupUi

    void retranslateUi(QWidget *MyPluginWidget)
    {
        MyPluginWidget->setWindowTitle(QApplication::translate("MyPluginWidget", "Form", Q_NULLPTR));
        cmd_x->setText(QApplication::translate("MyPluginWidget", "cmd_x (m)", Q_NULLPTR));
        cmd_y->setText(QApplication::translate("MyPluginWidget", "cmd_y (m)", Q_NULLPTR));
        cmd_z->setText(QApplication::translate("MyPluginWidget", "cmd_z (m)", Q_NULLPTR));
        GroupBox_Measured->setTitle(QApplication::translate("MyPluginWidget", "Measured", Q_NULLPTR));
        FK_x->setText(QApplication::translate("MyPluginWidget", "X", Q_NULLPTR));
        txt_joint1->setText(QApplication::translate("MyPluginWidget", "0.0", Q_NULLPTR));
        rad3->setText(QApplication::translate("MyPluginWidget", "rad", Q_NULLPTR));
        txt_joint2->setText(QApplication::translate("MyPluginWidget", "0.0", Q_NULLPTR));
        FK_ym->setText(QApplication::translate("MyPluginWidget", "m", Q_NULLPTR));
        rad2->setText(QApplication::translate("MyPluginWidget", "rad", Q_NULLPTR));
        label_24->setText(QApplication::translate("MyPluginWidget", "Joint 3", Q_NULLPTR));
        rad4->setText(QApplication::translate("MyPluginWidget", "rad", Q_NULLPTR));
        txt_joint4->setText(QApplication::translate("MyPluginWidget", "0.0", Q_NULLPTR));
        Joint1->setText(QApplication::translate("MyPluginWidget", "Joint 1", Q_NULLPTR));
        rad1->setText(QApplication::translate("MyPluginWidget", "rad", Q_NULLPTR));
        txt_joint3->setText(QApplication::translate("MyPluginWidget", "0.0", Q_NULLPTR));
        label_26->setText(QApplication::translate("MyPluginWidget", "Y", Q_NULLPTR));
        FK_xm->setText(QApplication::translate("MyPluginWidget", "m", Q_NULLPTR));
        Joint2->setText(QApplication::translate("MyPluginWidget", "Joint 2", Q_NULLPTR));
        txt_FK_z->setText(QApplication::translate("MyPluginWidget", "0.0", Q_NULLPTR));
        Joint4->setText(QApplication::translate("MyPluginWidget", "Joint 4", Q_NULLPTR));
        FK_zm->setText(QApplication::translate("MyPluginWidget", "m", Q_NULLPTR));
        txt_FK_y->setText(QApplication::translate("MyPluginWidget", "0.0", Q_NULLPTR));
        FK_z->setText(QApplication::translate("MyPluginWidget", "Z", Q_NULLPTR));
        txt_FK_x->setText(QApplication::translate("MyPluginWidget", "0.0", Q_NULLPTR));
        txt_Error_y->setText(QApplication::translate("MyPluginWidget", "0.0", Q_NULLPTR));
        label_30->setText(QApplication::translate("MyPluginWidget", "m", Q_NULLPTR));
        label_9->setText(QApplication::translate("MyPluginWidget", "Y", Q_NULLPTR));
        txt_Error_x->setText(QApplication::translate("MyPluginWidget", "0.0", Q_NULLPTR));
        label_29->setText(QApplication::translate("MyPluginWidget", "m", Q_NULLPTR));
        label_8->setText(QApplication::translate("MyPluginWidget", "Z", Q_NULLPTR));
        txt_Error_z->setText(QApplication::translate("MyPluginWidget", "0.0", Q_NULLPTR));
        label_28->setText(QApplication::translate("MyPluginWidget", "m", Q_NULLPTR));
        lbl_x->setText(QApplication::translate("MyPluginWidget", "X", Q_NULLPTR));
        label_27->setText(QApplication::translate("MyPluginWidget", "From FK", Q_NULLPTR));
        label_31->setText(QApplication::translate("MyPluginWidget", "Error Position", Q_NULLPTR));
        btn_Start->setText(QApplication::translate("MyPluginWidget", "Start !", Q_NULLPTR));
        lbl_cmd_x->setText(QApplication::translate("MyPluginWidget", "0.000", Q_NULLPTR));
        lbl_cmd_y->setText(QApplication::translate("MyPluginWidget", "0.000", Q_NULLPTR));
        lbl_cmd_z->setText(QApplication::translate("MyPluginWidget", "0.000", Q_NULLPTR));
        chk_Publish->setText(QApplication::translate("MyPluginWidget", "Publish Command", Q_NULLPTR));
        lbl_ping_update->setText(QApplication::translate("MyPluginWidget", "0.000", Q_NULLPTR));
        lbl_ping->setText(QApplication::translate("MyPluginWidget", "ping : ", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MyPluginWidget: public Ui_MyPluginWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MY_PLUGIN_H
