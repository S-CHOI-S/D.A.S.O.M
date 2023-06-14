/****************************************************************************
** Meta object code from reading C++ file 'my_plugin.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/rqt_mypkg_icros/include/my_plugin.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'my_plugin.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rqt_mypkg_cpp__MyPlugin_t {
    QByteArrayData data[24];
    char stringdata0[302];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rqt_mypkg_cpp__MyPlugin_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rqt_mypkg_cpp__MyPlugin_t qt_meta_stringdata_rqt_mypkg_cpp__MyPlugin = {
    {
QT_MOC_LITERAL(0, 0, 23), // "rqt_mypkg_cpp::MyPlugin"
QT_MOC_LITERAL(1, 24, 13), // "publisher_set"
QT_MOC_LITERAL(2, 38, 0), // ""
QT_MOC_LITERAL(3, 39, 15), // "ros::TimerEvent"
QT_MOC_LITERAL(4, 55, 12), // "callback_set"
QT_MOC_LITERAL(5, 68, 13), // "ping_callback"
QT_MOC_LITERAL(6, 82, 14), // "qsc_x_callback"
QT_MOC_LITERAL(7, 97, 3), // "val"
QT_MOC_LITERAL(8, 101, 14), // "qsc_y_callback"
QT_MOC_LITERAL(9, 116, 14), // "qsc_z_callback"
QT_MOC_LITERAL(10, 131, 8), // "writeLog"
QT_MOC_LITERAL(11, 140, 3), // "str"
QT_MOC_LITERAL(12, 144, 18), // "btn_Start_Callback"
QT_MOC_LITERAL(13, 163, 24), // "AngleSubscriber_Callback"
QT_MOC_LITERAL(14, 188, 23), // "sensor_msgs::JointState"
QT_MOC_LITERAL(15, 212, 3), // "msg"
QT_MOC_LITERAL(16, 216, 24), // "LimitSubscriber_Callback"
QT_MOC_LITERAL(17, 241, 25), // "rqt_mypkg::DasomDynamixel"
QT_MOC_LITERAL(18, 267, 2), // "DH"
QT_MOC_LITERAL(19, 270, 15), // "Eigen::Matrix4d"
QT_MOC_LITERAL(20, 286, 5), // "alpha"
QT_MOC_LITERAL(21, 292, 1), // "a"
QT_MOC_LITERAL(22, 294, 1), // "d"
QT_MOC_LITERAL(23, 296, 5) // "theta"

    },
    "rqt_mypkg_cpp::MyPlugin\0publisher_set\0"
    "\0ros::TimerEvent\0callback_set\0"
    "ping_callback\0qsc_x_callback\0val\0"
    "qsc_y_callback\0qsc_z_callback\0writeLog\0"
    "str\0btn_Start_Callback\0AngleSubscriber_Callback\0"
    "sensor_msgs::JointState\0msg\0"
    "LimitSubscriber_Callback\0"
    "rqt_mypkg::DasomDynamixel\0DH\0"
    "Eigen::Matrix4d\0alpha\0a\0d\0theta"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rqt_mypkg_cpp__MyPlugin[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   69,    2, 0x08 /* Private */,
       4,    1,   72,    2, 0x08 /* Private */,
       5,    1,   75,    2, 0x08 /* Private */,
       6,    1,   78,    2, 0x08 /* Private */,
       8,    1,   81,    2, 0x08 /* Private */,
       9,    1,   84,    2, 0x08 /* Private */,
      10,    1,   87,    2, 0x08 /* Private */,
      12,    1,   90,    2, 0x08 /* Private */,
      13,    1,   93,    2, 0x08 /* Private */,
      16,    1,   96,    2, 0x08 /* Private */,
      18,    4,   99,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    2,
    QMetaType::Void, 0x80000000 | 3,    2,
    QMetaType::Void, 0x80000000 | 3,    2,
    QMetaType::Void, QMetaType::Int,    7,
    QMetaType::Void, QMetaType::Int,    7,
    QMetaType::Void, QMetaType::Int,    7,
    QMetaType::Void, QMetaType::QString,   11,
    QMetaType::Void, QMetaType::Bool,    7,
    QMetaType::Void, 0x80000000 | 14,   15,
    QMetaType::Void, 0x80000000 | 17,   15,
    0x80000000 | 19, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,   20,   21,   22,   23,

       0        // eod
};

void rqt_mypkg_cpp::MyPlugin::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MyPlugin *_t = static_cast<MyPlugin *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->publisher_set((*reinterpret_cast< const ros::TimerEvent(*)>(_a[1]))); break;
        case 1: _t->callback_set((*reinterpret_cast< const ros::TimerEvent(*)>(_a[1]))); break;
        case 2: _t->ping_callback((*reinterpret_cast< const ros::TimerEvent(*)>(_a[1]))); break;
        case 3: _t->qsc_x_callback((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->qsc_y_callback((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->qsc_z_callback((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->writeLog((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: _t->btn_Start_Callback((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->AngleSubscriber_Callback((*reinterpret_cast< const sensor_msgs::JointState(*)>(_a[1]))); break;
        case 9: _t->LimitSubscriber_Callback((*reinterpret_cast< const rqt_mypkg::DasomDynamixel(*)>(_a[1]))); break;
        case 10: { Eigen::Matrix4d _r = _t->DH((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< Eigen::Matrix4d*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    }
}

const QMetaObject rqt_mypkg_cpp::MyPlugin::staticMetaObject = {
    { &rqt_gui_cpp::Plugin::staticMetaObject, qt_meta_stringdata_rqt_mypkg_cpp__MyPlugin.data,
      qt_meta_data_rqt_mypkg_cpp__MyPlugin,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *rqt_mypkg_cpp::MyPlugin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rqt_mypkg_cpp::MyPlugin::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rqt_mypkg_cpp__MyPlugin.stringdata0))
        return static_cast<void*>(this);
    return rqt_gui_cpp::Plugin::qt_metacast(_clname);
}

int rqt_mypkg_cpp::MyPlugin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rqt_gui_cpp::Plugin::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
