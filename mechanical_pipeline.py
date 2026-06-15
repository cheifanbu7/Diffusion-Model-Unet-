import os
import csv
import sys
import time
import random
import shutil
import argparse
import tempfile
import subprocess
from PIL import Image


# ---------------------------------------------------------------------
# User settings
# ---------------------------------------------------------------------

WORKDIR = r"C:\Users\User\Desktop\Project_1"
PMDB_PATH = os.path.join(WORKDIR, "2D_Plate.pmdb")

DATA_DIR = os.path.join(WORKDIR, "Data")
CSV_PATH = os.path.join(DATA_DIR, "metadata.csv")

NUMBER_OF_ITERATIONS = 100

RUN_WORKERS_IN_SUBPROCESSES = True
DELETE_ANSYS_FILES = True
OVERWRITE_EXISTING = False

RANDOM_SEED = None

MATERIAL_NAME = "Structural Steel"
THICKNESS = "1 [mm]"
ELEMENT_SIZE = "1 [mm]"

FORCE_MAGNITUDE = 100.0
FORCE_DIRECTIONS = [0, 90, 180, 270]

MASS_RETAIN_PERCENT = 75
RAW_EXPORT_SIZE = 1024
FINAL_IMAGE_SIZE = 256
APPLY_SMOOTHING = True


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def ensure_data_dir():
    os.makedirs(DATA_DIR, exist_ok=True)


def load_metadata_rows():
    if not os.path.isfile(CSV_PATH):
        return []

    with open(CSV_PATH, mode="r", newline="") as csv_file:
        return list(csv.DictReader(csv_file))


def get_next_serial_number():
    rows = load_metadata_rows()
    if not rows:
        return 1

    serials = []
    for row in rows:
        try:
            serials.append(int(row["serial_number"]))
        except Exception:
            pass

    return max(serials) + 1 if serials else 1


def write_metadata_row(row):
    file_exists = os.path.isfile(CSV_PATH)

    fieldnames = [
        "serial_number",
        "filename",
        "fixed_point_x",
        "fixed_point_y",
        "force_point_x",
        "force_point_y",
        "force_direction_degrees",
    ]

    with open(CSV_PATH, mode="a", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

        if not file_exists:
            writer.writeheader()

        writer.writerow(row)


def random_force_components(angle_degrees):
    if angle_degrees == 0:
        return FORCE_MAGNITUDE, 0.0
    if angle_degrees == 90:
        return 0.0, FORCE_MAGNITUDE
    if angle_degrees == 180:
        return -FORCE_MAGNITUDE, 0.0
    if angle_degrees == 270:
        return 0.0, -FORCE_MAGNITUDE

    raise ValueError(f"Unsupported force angle: {angle_degrees}")


def random_force_point_on_top_or_right():
    side = random.choice(["top", "right"])

    if side == "top":
        return random.randint(0, 10), 10

    return 10, random.randint(0, 10)


def generate_random_bc_set():
    while True:
        fixed_point_1 = (random.randint(0, 10), 0)
        fixed_point_2 = (0, random.randint(0, 10))
        force_point = random_force_point_on_top_or_right()

        points = [fixed_point_1, fixed_point_2, force_point]

        if len(set(points)) == 3:
            break

    force_angle = random.choice(FORCE_DIRECTIONS)
    force_fx, force_fy = random_force_components(force_angle)

    return fixed_point_1, fixed_point_2, force_point, force_angle, force_fx, force_fy


def make_centered_256_grayscale(input_png, output_png):
    image = Image.open(input_png).convert("L")
    image.thumbnail((FINAL_IMAGE_SIZE, FINAL_IMAGE_SIZE), Image.Resampling.LANCZOS)

    canvas = Image.new("L", (FINAL_IMAGE_SIZE, FINAL_IMAGE_SIZE), 255)

    x = (FINAL_IMAGE_SIZE - image.width) // 2
    y = (FINAL_IMAGE_SIZE - image.height) // 2

    canvas.paste(image, (x, y))
    canvas.save(output_png)


# ---------------------------------------------------------------------
# One isolated Mechanical case
# ---------------------------------------------------------------------

def run_one_case(
    serial_number,
    fixed_point_1,
    fixed_point_2,
    force_point,
    force_direction_degrees,
    force_fx,
    force_fy,
):
    ensure_data_dir()

    if not os.path.isfile(PMDB_PATH):
        raise FileNotFoundError(f"Missing geometry file: {PMDB_PATH}")

    filename = f"{serial_number}.png"
    output_png = os.path.join(DATA_DIR, filename)

    if os.path.isfile(output_png) and not OVERWRITE_EXISTING:
        print(f"Skipping existing {filename}")
        return

    run_dir = tempfile.mkdtemp(prefix=f"ansys_case_{serial_number}_", dir=WORKDIR)
    old_cwd = os.getcwd()

    app = None

    try:
        os.chdir(run_dir)

        import ansys.mechanical.core as mech
        import ansys.mechanical.core.embedding.app as mechanical_app_module

        for old_app in list(mechanical_app_module.INSTANCES):
            try:
                old_app.close()
            except Exception:
                try:
                    old_app._dispose()
                except Exception:
                    pass

        mechanical_app_module.INSTANCES.clear()

        app = mech.App(version=252, globals=globals())
        ExtAPI.Application.ActiveUnitSystem = MechanicalUnitSystem.StandardNMM

        geometry_import = Model.GeometryImportGroup.AddGeometryImport()
        geometry_import_format = (
            Ansys.Mechanical.DataModel.Enums.GeometryImportPreference.Format.Automatic
        )

        geometry_import_preferences = Ansys.ACT.Mechanical.Utilities.GeometryImportPreferences()
        geometry_import_preferences.ProcessNamedSelections = True
        geometry_import_preferences.NameSelectionKey = ""

        geometry_import.Import(
            str(PMDB_PATH),
            geometry_import_format,
            geometry_import_preferences,
        )

        def get_named_selection(name):
            for ns in Model.NamedSelections.Children:
                if ns.Name == name:
                    return ns
            raise RuntimeError(f"Named selection not found: {name}")


        def get_named_selection_count(ns):
            try:
                return ns.Location.Ids.Count
            except Exception:
                try:
                    return len(ns.Location.Ids)
                except Exception:
                    return None


        def validate_named_selection(ns, name):
            count = get_named_selection_count(ns)
            print(f"{name} selected count: {count}")

            if count == 0:
                raise RuntimeError(f"{name} selected zero mesh nodes.")


        def make_node_named_selection_2d(name, x_mm, y_mm):
            ns = Model.AddNamedSelection()
            ns.Name = name
            ns.ScopingMethod = GeometryDefineByType.Worksheet

            criteria = ns.GenerationCriteria

            cx = Ansys.ACT.Automation.Mechanical.NamedSelectionCriterion()
            cx.Active = True
            cx.Action = SelectionActionType.Add
            cx.EntityType = SelectionType.MeshNode
            cx.Criterion = SelectionCriterionType.LocationX
            cx.Operator = SelectionOperatorType.Equal
            cx.Value = Quantity(f"{x_mm} [mm]")
            criteria.Add(cx)

            cy = Ansys.ACT.Automation.Mechanical.NamedSelectionCriterion()
            cy.Active = True
            cy.Action = SelectionActionType.Filter
            cy.EntityType = SelectionType.MeshNode
            cy.Criterion = SelectionCriterionType.LocationY
            cy.Operator = SelectionOperatorType.Equal
            cy.Value = Quantity(f"{y_mm} [mm]")
            criteria.Add(cy)

            try:
                ns.ZeroTolerance = 1e-8
            except Exception:
                pass

            ns.Generate()
            validate_named_selection(ns, name)

            return ns


        def hide_graphics_overlays():
            candidates = [
                ExtAPI.Graphics,
                getattr(ExtAPI.Graphics, "ViewOptions", None),
                getattr(ExtAPI.Graphics, "GlobalLegendSettings", None),
                getattr(ExtAPI.Graphics, "ResultPreference", None),
                getattr(ExtAPI.Graphics, "ModelDisplay", None),
            ]

            attrs = [
                "ShowLegend",
                "ShowTriad",
                "ShowRuler",
                "ShowScale",
                "ShowScaleBar",
                "ShowAnnotations",
                "ShowMinMax",
                "ShowDateTime",
                "ShowTitle",
                "ShowLogo",
                "ShowProbeLabels",
                "ShowCoordinateTriad",
            ]

            for obj in candidates:
                if obj is None:
                    continue
                for attr in attrs:
                    try:
                        setattr(obj, attr, False)
                    except Exception:
                        pass


        def export_clean_png(raw_png_path):
            hide_graphics_overlays()

            image_format = GraphicsImageExportFormat.PNG

            settings = Ansys.Mechanical.Graphics.GraphicsImageExportSettings()
            settings.Resolution = GraphicsResolutionType.EnhancedResolution
            settings.Background = GraphicsBackgroundType.White
            settings.Width = RAW_EXPORT_SIZE
            settings.Height = RAW_EXPORT_SIZE
            settings.CurrentGraphicsDisplay = False

            try:
                settings.Capture = GraphicsCaptureType.ImageOnly
            except Exception:
                pass

            ExtAPI.Graphics.Camera.SetSpecificViewOrientation(ViewOrientationType.Front)
            ExtAPI.Graphics.Camera.SetFit()
            time.sleep(2.0)

            ExtAPI.Graphics.ExportImage(str(raw_png_path), image_format, settings)


        plate_ns = get_named_selection("Plate")

        geo_body = Model.Geometry.Children[0].Children[0]

        try:
            geo_body.Dimension = ShellBodyDimension.Two_D
        except Exception:
            pass

        geo_body.Material = MATERIAL_NAME
        geo_body.Thickness = Quantity(THICKNESS)

        mesh = Model.Mesh
        mesh.ElementSize = Quantity(ELEMENT_SIZE)
        mesh.GenerateMesh()

        fixed_point_1_ns = make_node_named_selection_2d(
            "Fixed_Point_1",
            fixed_point_1[0],
            fixed_point_1[1],
        )

        fixed_point_2_ns = make_node_named_selection_2d(
            "Fixed_Point_2",
            fixed_point_2[0],
            fixed_point_2[1],
        )

        force_point_ns = make_node_named_selection_2d(
            "Force_Point",
            force_point[0],
            force_point[1],
        )

        static_analysis = Model.AddStaticStructuralAnalysis()

        fixed_1 = static_analysis.AddFixedSupport()
        fixed_1.Location = fixed_point_1_ns

        fixed_2 = static_analysis.AddFixedSupport()
        fixed_2.Location = fixed_point_2_ns

        force = static_analysis.AddForce()
        force.Location = force_point_ns
        force.DefineBy = LoadDefineBy.Components
        force.XComponent.Output.DiscreteValues = [Quantity(f"{force_fx} [N]")]
        force.YComponent.Output.DiscreteValues = [Quantity(f"{force_fy} [N]")]

        static_solution = static_analysis.Solution
        static_solution.AddTotalDeformation()
        static_solution.AddEquivalentStress()
        static_solution.Solve(True)

        topo_analysis = Model.AddTopologyOptimizationAnalysis()
        topo_analysis.TransferDataFrom(static_analysis)

        optimization_regions = ExtAPI.DataModel.GetObjectsByType(
            DataModelObjectCategory.OptimizationRegion
        )

        optimization_region = optimization_regions[optimization_regions.Count - 1]
        optimization_region.OptimizationType = OptimizationType.TopologyDensity
        optimization_region.BoundaryCondition = BoundaryConditionType.AllLoadsAndSupports

        try:
            optimization_region.Location = plate_ns
        except Exception as exc:
            print(f"Could not assign Plate named selection directly: {exc}")

        topo_analysis.AddObjective = ObjectiveType.MinimizeCompliance

        mass_constraint = topo_analysis.Children[3]
        mass_constraint.PercentageToRetain = MASS_RETAIN_PERCENT

        topo_solution = topo_analysis.Solution
        topo_solution.Solve(True)

        topology_density = topo_solution.Children[1]
        topology_density.Activate()

        if APPLY_SMOOTHING:
            try:
                topology_density.AddSmoothing()
                topo_solution.EvaluateAllResults()
                topology_density.Children[0].Activate()
            except Exception as exc:
                print(f"Smoothing failed: {exc}")

        raw_png = os.path.join(run_dir, f"raw_{filename}")
        export_clean_png(raw_png)
        make_centered_256_grayscale(raw_png, output_png)

        write_metadata_row(
            {
                "serial_number": serial_number,
                "filename": filename,
                "fixed_point_x": fixed_point_1[0],
                "fixed_point_y": fixed_point_2[1],
                "force_point_x": force_point[0],
                "force_point_y": force_point[1],
                "force_direction_degrees": force_direction_degrees,
            }
        )

        print(f"Saved PNG: {output_png}")

    finally:
        if app is not None:
            try:
                app.close()
            except Exception:
                try:
                    app._dispose()
                except Exception:
                    pass

        os.chdir(old_cwd)

        if DELETE_ANSYS_FILES and os.path.isdir(run_dir):
            shutil.rmtree(run_dir, ignore_errors=True)
        elif os.path.isdir(run_dir):
            print(f"Kept ANSYS run folder: {run_dir}")


# ---------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------

def run_controller():
    ensure_data_dir()

    if RANDOM_SEED is not None:
        random.seed(RANDOM_SEED)

    next_serial = get_next_serial_number()

    for i in range(NUMBER_OF_ITERATIONS):
        serial_number = next_serial + i

        (
            fixed_point_1,
            fixed_point_2,
            force_point,
            force_angle,
            force_fx,
            force_fy,
        ) = generate_random_bc_set()

        print(
            f"Running serial {serial_number}: "
            f"FP1={fixed_point_1}, FP2={fixed_point_2}, "
            f"ForcePoint={force_point}, ForceDirection={force_angle} deg"
        )

        if RUN_WORKERS_IN_SUBPROCESSES:
            script_path = os.path.abspath(__file__)

            cmd = [
                sys.executable,
                script_path,
                "--worker",
                "--serial",
                str(serial_number),
                "--fp1x",
                str(fixed_point_1[0]),
                "--fp1y",
                str(fixed_point_1[1]),
                "--fp2x",
                str(fixed_point_2[0]),
                "--fp2y",
                str(fixed_point_2[1]),
                "--force-x",
                str(force_point[0]),
                "--force-y",
                str(force_point[1]),
                "--angle",
                str(force_angle),
                "--fx",
                str(force_fx),
                "--fy",
                str(force_fy),
            ]

            result = subprocess.run(cmd, cwd=WORKDIR)

            if result.returncode != 0:
                print(f"Serial {serial_number} failed with return code {result.returncode}")
                continue

        else:
            run_one_case(
                serial_number=serial_number,
                fixed_point_1=fixed_point_1,
                fixed_point_2=fixed_point_2,
                force_point=force_point,
                force_direction_degrees=force_angle,
                force_fx=force_fx,
                force_fy=force_fy,
            )


# ---------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------

parser = argparse.ArgumentParser()

parser.add_argument("--worker", action="store_true")
parser.add_argument("--serial", type=int)

parser.add_argument("--fp1x", type=int)
parser.add_argument("--fp1y", type=int)
parser.add_argument("--fp2x", type=int)
parser.add_argument("--fp2y", type=int)

parser.add_argument("--force-x", type=int)
parser.add_argument("--force-y", type=int)

parser.add_argument("--angle", type=int)
parser.add_argument("--fx", type=float)
parser.add_argument("--fy", type=float)

args, _unknown = parser.parse_known_args()

if args.worker:
    run_one_case(
        serial_number=args.serial,
        fixed_point_1=(args.fp1x, args.fp1y),
        fixed_point_2=(args.fp2x, args.fp2y),
        force_point=(args.force_x, args.force_y),
        force_direction_degrees=args.angle,
        force_fx=args.fx,
        force_fy=args.fy,
    )
else:
    run_controller()