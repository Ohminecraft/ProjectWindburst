"""The name a USB serial device gives itself (its iProduct string), per port.

Windows shows every CDC port as "USB Serial Device (COM5)" - the name of its driver, not of the device: pyserial's description
is that, and its `product` stays empty there. The device's own string is in the device property "Bus reported device
description" (DEVPKEY_Device_BusReportedDeviceDesc), which the USB stack fills from the descriptor. For a composite device
(native USB of an ESP32-S3 / STM32 with TinyUSB: the port is interface MI_00 of it) the interface's node may carry the
interface string or nothing; the device node above it carries iProduct - that one is taken.
On Linux / macOS pyserial already reads iProduct (ListPortInfo.product).

    product_names() -> {"COM5": "Windburst ESP32-S3", ...}     (empty where nothing can be read; never raises)
"""
import sys


def product_names():
    if sys.platform != "win32":
        return {}
    try:
        return _windows()
    except Exception:  # noqa: BLE001 - a name is a nicety: never a reason for the port list to fail
        return {}


def _windows():
    import ctypes
    from ctypes import wintypes

    setupapi = ctypes.WinDLL("setupapi", use_last_error=True)
    cfgmgr = ctypes.WinDLL("cfgmgr32")
    advapi = ctypes.WinDLL("advapi32")

    class GUID(ctypes.Structure):
        _fields_ = [("Data1", wintypes.DWORD), ("Data2", wintypes.WORD), ("Data3", wintypes.WORD), ("Data4", ctypes.c_ubyte * 8)]

    class SP_DEVINFO_DATA(ctypes.Structure):
        _fields_ = [("cbSize", wintypes.DWORD), ("ClassGuid", GUID), ("DevInst", wintypes.DWORD), ("Reserved", ctypes.c_void_p)]

    class DEVPROPKEY(ctypes.Structure):
        _fields_ = [("fmtid", GUID), ("pid", wintypes.ULONG)]

    def guid(a, b, c, d):
        g = GUID(a, b, c)
        for i, x in enumerate(d):
            g.Data4[i] = x
        return g

    PORTS = guid(0x4D36E978, 0xE325, 0x11CE, (0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18))           # GUID_DEVCLASS_PORTS
    BUS_DESC = DEVPROPKEY(guid(0x540B947E, 0x8B40, 0x45BC, (0xA8, 0xA2, 0x6A, 0x0B, 0x89, 0x4C, 0xBD, 0xA2)), 4)    # DEVPKEY_Device_BusReportedDeviceDesc
    DIGCF_PRESENT, DICS_FLAG_GLOBAL, DIREG_DEV, KEY_READ = 0x2, 0x1, 0x1, 0x20019
    INVALID = ctypes.c_void_p(-1).value

    setupapi.SetupDiGetClassDevsW.restype = ctypes.c_void_p
    setupapi.SetupDiGetClassDevsW.argtypes = [ctypes.POINTER(GUID), wintypes.LPCWSTR, wintypes.HWND, wintypes.DWORD]
    setupapi.SetupDiEnumDeviceInfo.argtypes = [ctypes.c_void_p, wintypes.DWORD, ctypes.POINTER(SP_DEVINFO_DATA)]
    setupapi.SetupDiOpenDevRegKey.restype = ctypes.c_void_p
    setupapi.SetupDiOpenDevRegKey.argtypes = [ctypes.c_void_p, ctypes.POINTER(SP_DEVINFO_DATA), wintypes.DWORD, wintypes.DWORD, wintypes.DWORD, wintypes.DWORD]
    setupapi.SetupDiGetDeviceInstanceIdW.argtypes = [ctypes.c_void_p, ctypes.POINTER(SP_DEVINFO_DATA), wintypes.LPWSTR, wintypes.DWORD, ctypes.POINTER(wintypes.DWORD)]
    setupapi.SetupDiDestroyDeviceInfoList.argtypes = [ctypes.c_void_p]
    advapi.RegQueryValueExW.argtypes = [ctypes.c_void_p, wintypes.LPCWSTR, ctypes.c_void_p, ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p, ctypes.POINTER(wintypes.DWORD)]
    advapi.RegCloseKey.argtypes = [ctypes.c_void_p]
    cfgmgr.CM_Get_DevNode_PropertyW.argtypes = [wintypes.DWORD, ctypes.POINTER(DEVPROPKEY), ctypes.POINTER(wintypes.ULONG), ctypes.c_void_p, ctypes.POINTER(wintypes.ULONG), wintypes.ULONG]
    cfgmgr.CM_Get_Parent.argtypes = [ctypes.POINTER(wintypes.DWORD), wintypes.DWORD, wintypes.ULONG]

    def node_text(devinst):
        ptype, size = wintypes.ULONG(0), wintypes.ULONG(0)
        cfgmgr.CM_Get_DevNode_PropertyW(devinst, ctypes.byref(BUS_DESC), ctypes.byref(ptype), None, ctypes.byref(size), 0)
        if not size.value:
            return ""
        buf = ctypes.create_string_buffer(size.value)
        if cfgmgr.CM_Get_DevNode_PropertyW(devinst, ctypes.byref(BUS_DESC), ctypes.byref(ptype), buf, ctypes.byref(size), 0) != 0:
            return ""
        return buf.raw.decode("utf-16-le", "replace").split("\x00")[0].strip()

    out = {}
    h = setupapi.SetupDiGetClassDevsW(ctypes.byref(PORTS), None, None, DIGCF_PRESENT)
    if not h or h == INVALID:
        return out
    try:
        i = 0
        while True:
            d = SP_DEVINFO_DATA()
            d.cbSize = ctypes.sizeof(SP_DEVINFO_DATA)
            if not setupapi.SetupDiEnumDeviceInfo(h, i, ctypes.byref(d)):
                break
            i += 1
            key = setupapi.SetupDiOpenDevRegKey(h, ctypes.byref(d), DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ)
            if not key or key == INVALID:
                continue
            try:
                buf = ctypes.create_unicode_buffer(64)
                n = wintypes.DWORD(ctypes.sizeof(buf))
                if advapi.RegQueryValueExW(key, "PortName", None, None, buf, ctypes.byref(n)) != 0:
                    continue
                port = buf.value
            finally:
                advapi.RegCloseKey(key)
            ident = ctypes.create_unicode_buffer(512)
            setupapi.SetupDiGetDeviceInstanceIdW(h, ctypes.byref(d), ident, 512, None)
            name = ""
            if ident.value.upper().startswith("USB\\") and "&MI_" in ident.value.upper():      # an interface of a composite device: its parent is the device
                parent = wintypes.DWORD(0)
                if cfgmgr.CM_Get_Parent(ctypes.byref(parent), d.DevInst, 0) == 0:
                    name = node_text(parent.value)
            if not name:
                name = node_text(d.DevInst)
            if name:
                out[port] = name
    finally:
        setupapi.SetupDiDestroyDeviceInfoList(h)
    return out
