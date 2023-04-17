/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/dynamixel-workbench/dynamixel_workbench_single_manager_gui/include/dynamixel_workbench_single_manager_gui/main_window.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_main_window__MainWindow_t {
    QByteArrayData data[18];
    char stringdata0[398];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_main_window__MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_main_window__MainWindow_t qt_meta_stringdata_main_window__MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 23), // "main_window::MainWindow"
QT_MOC_LITERAL(1, 24, 24), // "on_actionAbout_triggered"
QT_MOC_LITERAL(2, 49, 0), // ""
QT_MOC_LITERAL(3, 50, 38), // "on_torque_enable_toggle_butto..."
QT_MOC_LITERAL(4, 89, 5), // "check"
QT_MOC_LITERAL(5, 95, 29), // "on_reboot_push_button_clicked"
QT_MOC_LITERAL(6, 125, 36), // "on_factory_reset_push_button_..."
QT_MOC_LITERAL(7, 162, 40), // "on_set_position_zero_push_but..."
QT_MOC_LITERAL(8, 203, 8), // "changeID"
QT_MOC_LITERAL(9, 212, 14), // "changeBaudrate"
QT_MOC_LITERAL(10, 227, 19), // "changeOperatingMode"
QT_MOC_LITERAL(11, 247, 23), // "changeControlTableValue"
QT_MOC_LITERAL(12, 271, 22), // "setEachAddressFunction"
QT_MOC_LITERAL(13, 294, 5), // "index"
QT_MOC_LITERAL(14, 300, 27), // "updateDynamixelInfoLineEdit"
QT_MOC_LITERAL(15, 328, 39), // "dynamixel_workbench_msgs::Dyn..."
QT_MOC_LITERAL(16, 368, 14), // "dynamixel_info"
QT_MOC_LITERAL(17, 383, 14) // "updateListView"

    },
    "main_window::MainWindow\0"
    "on_actionAbout_triggered\0\0"
    "on_torque_enable_toggle_button_toggled\0"
    "check\0on_reboot_push_button_clicked\0"
    "on_factory_reset_push_button_clicked\0"
    "on_set_position_zero_push_button_clicked\0"
    "changeID\0changeBaudrate\0changeOperatingMode\0"
    "changeControlTableValue\0setEachAddressFunction\0"
    "index\0updateDynamixelInfoLineEdit\0"
    "dynamixel_workbench_msgs::DynamixelInfo\0"
    "dynamixel_info\0updateListView"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_main_window__MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   74,    2, 0x0a /* Public */,
       3,    1,   75,    2, 0x0a /* Public */,
       5,    1,   78,    2, 0x0a /* Public */,
       6,    1,   81,    2, 0x0a /* Public */,
       7,    1,   84,    2, 0x0a /* Public */,
       8,    0,   87,    2, 0x0a /* Public */,
       9,    0,   88,    2, 0x0a /* Public */,
      10,    0,   89,    2, 0x0a /* Public */,
      11,    0,   90,    2, 0x0a /* Public */,
      12,    1,   91,    2, 0x0a /* Public */,
      14,    1,   94,    2, 0x0a /* Public */,
      17,    0,   97,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   13,
    QMetaType::Void, 0x80000000 | 15,   16,
    QMetaType::Void,

       0        // eod
};

void main_window::MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_actionAbout_triggered(); break;
        case 1: _t->on_torque_enable_toggle_button_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->on_reboot_push_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->on_factory_reset_push_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->on_set_position_zero_push_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->changeID(); break;
        case 6: _t->changeBaudrate(); break;
        case 7: _t->changeOperatingMode(); break;
        case 8: _t->changeControlTableValue(); break;
        case 9: _t->setEachAddressFunction((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 10: _t->updateDynamixelInfoLineEdit((*reinterpret_cast< dynamixel_workbench_msgs::DynamixelInfo(*)>(_a[1]))); break;
        case 11: _t->updateListView(); break;
        default: ;
        }
    }
}

const QMetaObject main_window::MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_main_window__MainWindow.data,
      qt_meta_data_main_window__MainWindow,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *main_window::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *main_window::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_main_window__MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int main_window::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
