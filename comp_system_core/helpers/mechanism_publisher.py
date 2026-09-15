"""Publish a Mechanism2d to NetworkTables by hand, because 2027a7 cannot.

WHY THIS EXISTS
---------------
On a7 a Mechanism2d cannot be published through the telemetry registry from Python at all.
It exposes log_to(_NativeTelemetryTable) and nothing hands Python one of those:

    TelemetryRegistry.get_table(...)  -> telemetry.TelemetryTable   (wrong type, TypeError)
    _NativeTelemetryTable(path)       -> TypeError: No constructor defined!

That is a binding gap in the alpha, not something our code is doing wrong.  Until it is
fixed the mech view is simply dark, so this module reproduces the wire format instead.

THE WIRE FORMAT was not guessed.  It was captured by publishing a Mechanism2d from the a6
environment - which still had SmartDashboard - and dumping every NetworkTables entry:

    /SmartDashboard/Mech/.type             string   "Mechanism2d"
    /SmartDashboard/Mech/.name             string   "Mech"
    /SmartDashboard/Mech/dims              double[] [width, height]
    /SmartDashboard/Mech/backgroundColor   string   "#141E28"
    /SmartDashboard/Mech/<root>/x          double
    /SmartDashboard/Mech/<root>/y          double
    /SmartDashboard/Mech/<root>/<lig>/.type    string  "line"
    /SmartDashboard/Mech/<root>/<lig>/angle    double
    /SmartDashboard/Mech/<root>/<lig>/color    string  "#FF0000"
    /SmartDashboard/Mech/<root>/<lig>/length   double
    /SmartDashboard/Mech/<root>/<lig>/weight   double
    ... nested ligaments as further subtables under their parent

Note roots have NO .type key - only ligaments do.  That asymmetry is real; adding one makes
the dashboard treat the root as a line.

WHY THE TREE HAS TO BE RECORDED
-------------------------------
a7's Mechanism2d and MechanismRoot2d cannot be walked: Mechanism2d has only get_root(),
MechanismRoot2d has only get_name(), and neither can list its children.  The per-loop VALUES
are readable (MechanismLigament2d has get_angle / get_length / get_color / get_line_weight),
but the STRUCTURE is not.

So install() wraps the four builder methods and records the tree as it is constructed.  This
means blockhead_mech.py - 43 ligaments nested several levels deep - needs no changes at all.
Root x/y are recorded the same way, because MechanismRoot2d has set_position() but no getter.

install() must run BEFORE any Mechanism2d is built.  dashboard.install() calls it, and that
runs at import time in robot.py, well before BlockheadMech is constructed.

DELETE THIS FILE when log_to() starts working - see _NATIVE_GAP in helpers/dashboard.py.
"""

import ntcore
import wpilib

# id(obj) -> record.  The record holds a strong reference to the object itself, so the id
# stays valid and cannot be recycled onto a different object.
_mechs: dict[int, dict] = {}
_nodes: dict[int, dict] = {}
_installed = False


def _hex(color) -> str:
    """Color8Bit -> '#RRGGBB'.

    hex_string is a METHOD on a7, not a property - forgetting the parens hands NT a bound
    method object instead of a string, and the TypeError names put_string rather than this
    function, which makes it a confusing five minutes.
    """
    fn = getattr(color, "hex_string", None)
    if callable(fn):
        return fn()
    if isinstance(fn, str):                    # property in some binding versions
        return fn
    return f"#{color.red:02X}{color.green:02X}{color.blue:02X}"


def install() -> None:
    """Wrap the Mechanism2d builders so the tree can be reconstructed later."""
    global _installed
    if _installed:
        return

    mech_init = wpilib.Mechanism2d.__init__
    mech_root = wpilib.Mechanism2d.get_root
    mech_bg = wpilib.Mechanism2d.set_background_color
    root_append = wpilib.MechanismRoot2d.append_ligament
    root_setpos = wpilib.MechanismRoot2d.set_position
    lig_append = wpilib.MechanismLigament2d.append_ligament

    def _init(self, width, height, *a, **k):
        mech_init(self, width, height, *a, **k)
        _mechs[id(self)] = {"obj": self, "dims": [float(width), float(height)],
                            "bg": "#000020", "roots": []}

    def _get_root(self, name, x, y):
        root = mech_root(self, name, x, y)
        rec = _mechs.get(id(self))
        node = _nodes.get(id(root))
        if node is None:
            node = {"obj": root, "name": name, "x": float(x), "y": float(y), "children": []}
            _nodes[id(root)] = node
            if rec is not None:
                rec["roots"].append(node)
        else:                                   # get_root is create-or-get; refresh position
            node["x"], node["y"] = float(x), float(y)
        return root

    def _set_bg(self, color):
        mech_bg(self, color)
        if id(self) in _mechs:
            _mechs[id(self)]["bg"] = _hex(color)

    def _set_pos(self, x, y):
        root_setpos(self, x, y)
        node = _nodes.get(id(self))
        if node is not None:
            node["x"], node["y"] = float(x), float(y)

    def _append(original):
        def wrapper(self, *a, **k):
            child = original(self, *a, **k)
            parent = _nodes.get(id(self))
            node = {"obj": child, "name": child.get_name(), "children": []}
            _nodes[id(child)] = node
            if parent is not None:
                parent["children"].append(node)
            return child
        return wrapper

    wpilib.Mechanism2d.__init__ = _init
    wpilib.Mechanism2d.get_root = _get_root
    wpilib.Mechanism2d.set_background_color = _set_bg
    wpilib.MechanismRoot2d.set_position = _set_pos
    wpilib.MechanismRoot2d.append_ligament = _append(root_append)
    wpilib.MechanismLigament2d.append_ligament = _append(lig_append)
    _installed = True


# Published-value cache, so we only write a key when it actually changes.  The real Sendable
# behaved this way; without it 43 ligaments x 4 values x 50 Hz is ~8600 NT writes a second.
_last: dict[str, object] = {}


def _put(table, key, value, kind: str) -> None:
    path = f"{table.get_path()}/{key}"
    if _last.get(path) == value:
        return
    _last[path] = value
    if kind == "d":
        table.put_number(key, value)
    elif kind == "s":
        table.put_string(key, value)
    else:
        table.put_number_array(key, value)


def _publish_node(table, node: dict, is_root: bool) -> None:
    sub = table.get_sub_table(node["name"])
    if is_root:
        # Roots carry only x/y - deliberately no .type, see the module docstring.
        _put(sub, "x", node["x"], "d")
        _put(sub, "y", node["y"], "d")
    else:
        lig = node["obj"]
        _put(sub, ".type", "line", "s")
        _put(sub, "angle", lig.get_angle(), "d")
        _put(sub, "length", lig.get_length(), "d")
        _put(sub, "weight", lig.get_line_weight(), "d")
        _put(sub, "color", _hex(lig.get_color()), "s")
    for child in node["children"]:
        _publish_node(sub, child, is_root=False)


def publish(key: str, mech) -> bool:
    """Write `mech` to NetworkTables under `key`.  False if install() missed its creation."""
    rec = _mechs.get(id(mech))
    if rec is None:
        return False
    table = ntcore.NetworkTableInstance.get_default().get_table(key.lstrip("/"))
    name = key.rstrip("/").rpartition("/")[2]
    _put(table, ".type", "Mechanism2d", "s")
    _put(table, ".name", name, "s")
    _put(table, "dims", rec["dims"], "a")
    _put(table, "backgroundColor", rec["bg"], "s")
    for root in rec["roots"]:
        _publish_node(table, root, is_root=True)
    rec["key"] = key
    return True


def update() -> None:
    """Re-publish every registered mechanism.  Call each loop; unlike a real Sendable these
    do not update themselves."""
    for rec in _mechs.values():
        if "key" in rec:
            publish(rec["key"], rec["obj"])
