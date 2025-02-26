from PySide6 import QtWidgets, QtCore, QtGui
import sys
import json
import shapely.geometry as sg
from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
import matplotlib.pyplot as plt
from matplotlib.figure import Figure


class MyWidget(QtWidgets.QWidget):
    def __init__(self, redraw_func=None):
        super().__init__()
        self.table = QtWidgets.QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["X", "Y"])
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.addWidget(self.table)

        # Add row and remove row buttons for coordinate entry.
        self.add_row_button = QtWidgets.QPushButton("Add row")
        self.remove_row_button = QtWidgets.QPushButton("Remove row")
        self.layout.addWidget(self.add_row_button)
        self.layout.addWidget(self.remove_row_button)
        self.add_row_button.clicked.connect(self.add_row)
        self.remove_row_button.clicked.connect(self.remove_row)

        # If a callback is provided, update the plot on table changes.
        if redraw_func is not None:
            self.table.cellChanged.connect(redraw_func)
            self.remove_row_button.clicked.connect(redraw_func)

    def add_row(self):
        self.table.setRowCount(self.table.rowCount() + 1)

    def remove_row(self):
        current_rows = self.table.rowCount()
        if current_rows > 0:
            self.table.setRowCount(current_rows - 1)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Map Generator with Cost Zones")
        self.setGeometry(100, 100, 800, 600)

        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QtWidgets.QVBoxLayout(self.central_widget)

        # Tabs for obstacles and cost zones.
        self.tabs = QtWidgets.QTabWidget()
        self.layout.addWidget(self.tabs)

        # Action buttons.
        self.make_polygon_button = QtWidgets.QPushButton("Add Obstacle")
        self.add_cost_zone_button = QtWidgets.QPushButton("Add Cost Zone")
        self.remove_zone_button = QtWidgets.QPushButton("Remove Selected Zone")
        self.copy_vertices_button = QtWidgets.QPushButton("Copy Vertices")
        self.export_map_button = QtWidgets.QPushButton("Export Map")
        self.import_map_button = QtWidgets.QPushButton("Import Map")

        self.layout.addWidget(self.make_polygon_button)
        self.layout.addWidget(self.add_cost_zone_button)
        self.layout.addWidget(self.remove_zone_button)
        self.layout.addWidget(self.copy_vertices_button)
        self.layout.addWidget(self.export_map_button)
        self.layout.addWidget(self.import_map_button)

        # Connect buttons to functions.
        self.make_polygon_button.clicked.connect(self.add_polygon)
        self.add_cost_zone_button.clicked.connect(self.add_cost_zone)
        self.remove_zone_button.clicked.connect(self.remove_zone)
        self.copy_vertices_button.clicked.connect(self.copy_vertices)
        self.export_map_button.clicked.connect(self.export_map)
        self.import_map_button.clicked.connect(self.import_map)

        # Input fields for plot limits and robot radius.
        self.field_x = QtWidgets.QLineEdit()
        self.field_x.setPlaceholderText("Field X")
        self.field_y = QtWidgets.QLineEdit()
        self.field_y.setPlaceholderText("Field Y")
        self.robot_radius = QtWidgets.QLineEdit()
        self.robot_radius.setPlaceholderText("Robot Radius")
        self.layout.addWidget(self.field_x)
        self.layout.addWidget(self.field_y)
        self.layout.addWidget(self.robot_radius)

        # Allow only numeric (double) inputs.
        self.field_x.setValidator(QtGui.QDoubleValidator())
        self.field_y.setValidator(QtGui.QDoubleValidator())
        self.robot_radius.setValidator(QtGui.QDoubleValidator())
        self.field_x.textEdited.connect(self.regenerate_plot)
        self.field_y.textEdited.connect(self.regenerate_plot)
        self.robot_radius.textEdited.connect(self.regenerate_plot)

        # Plotting area using matplotlib.
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.plot = FigureCanvasQTAgg(self.fig)
        self.layout.addWidget(self.plot)

        # Counters for naming tabs.
        self.n_obstacles = 0
        self.n_cost_zones = 0

    def add_polygon(self):
        polygon_widget = MyWidget(self.regenerate_plot)
        tab_name = f"Obstacle {self.n_obstacles}"
        self.tabs.addTab(polygon_widget, tab_name)
        self.tabs.setTabToolTip(self.tabs.count() - 1, "Obstacle")
        self.n_obstacles += 1
        self.regenerate_plot()

    def add_cost_zone(self):
        cost_zone_widget = MyWidget(self.regenerate_plot)
        # Add a separate input for cost value.
        cost_zone_widget.cost_input = QtWidgets.QLineEdit()
        cost_zone_widget.cost_input.setPlaceholderText("Cost")
        cost_zone_widget.layout.addWidget(cost_zone_widget.cost_input)
        cost_zone_widget.cost_input.textChanged.connect(self.regenerate_plot)

        tab_name = f"Cost Zone {self.n_cost_zones}"
        self.tabs.addTab(cost_zone_widget, tab_name)
        self.tabs.setTabToolTip(self.tabs.count() - 1, "Cost Zone")
        self.n_cost_zones += 1
        self.regenerate_plot()

    def remove_zone(self):
        current_index = self.tabs.currentIndex()
        if current_index != -1:
            self.tabs.removeTab(current_index)
            self.regenerate_plot()

    def copy_vertices(self):
        
        output = ("// Auto-generated list of buffered polygon vertices with merged obstacles "
                  "and subtracted cost zones\n\n")
        try:
            robot_radius = float(self.robot_radius.text())
        except ValueError:
            robot_radius = 0.0

        # --- Process obstacles ---
        obstacle_polygons = []
        for tab_index in range(self.tabs.count()):
            if self.tabs.tabToolTip(tab_index) == "Obstacle":
                widget = self.tabs.widget(tab_index)
                table = widget.table
                vertices = []
                for row in range(table.rowCount()):
                    item_x = table.item(row, 0)
                    item_y = table.item(row, 1)
                    if item_x and item_y:
                        try:
                            x, y = float(item_x.text()), float(item_y.text())
                            vertices.append((x, y))
                        except ValueError:
                            continue
                if vertices:
                    poly = sg.Polygon(vertices)
                    if poly.is_valid:
                        obstacle_polygons.append(poly)
        # Use a small buffer to "fuzz" boundaries, then union and reverse buffer.
        if obstacle_polygons:
            tolerance = 0.001
            merged_obstacles = unary_union([p.buffer(tolerance) for p in obstacle_polygons]).buffer(-tolerance)
        else:
            merged_obstacles = None

        # Output merged obstacles.
        obstacle_index = 0
        if merged_obstacles is not None and (not merged_obstacles.is_empty):
            if merged_obstacles.geom_type == "Polygon":
                buffered_poly = merged_obstacles.buffer(robot_radius, cap_style=2, resolution=2)
                x_coords, y_coords = buffered_poly.exterior.xy
                output += f"vector<Point> obstacle_{obstacle_index} = {{\n"
                for x, y in zip(x_coords, y_coords):
                    if (output.find(f"  Point({x:.4f}, {y:.4f}),") == -1):
                        output += f"  Point({x:.4f}, {y:.4f}),\n"
                output += "};\n\n"
                obstacle_index += 1
            elif merged_obstacles.geom_type == "MultiPolygon":
                for poly in merged_obstacles.geoms:
                    buffered_poly = poly.buffer(robot_radius, cap_style=2, resolution=2)
                    x_coords, y_coords = buffered_poly.exterior.xy
                    output += f"vector<Point> obstacle_{obstacle_index} = {{\n"
                    for x, y in zip(x_coords, y_coords):
                        if (output.find(f"  Point({x:.4f}, {y:.4f}),") == -1):
                            output += f"  Point({x:.4f}, {y:.4f}),\n"
                    output += "};\n\n"
                    obstacle_index += 1

        # --- Process cost zones ---
        cost_zone_index = 0
        for tab_index in range(self.tabs.count()):
            if self.tabs.tabToolTip(tab_index) == "Cost Zone":
                widget = self.tabs.widget(tab_index)
                table = widget.table
                vertices = []
                for row in range(table.rowCount()):
                    try:
                        x = float(table.item(row, 0).text())
                        y = float(table.item(row, 1).text())
                        vertices.append((x, y))
                    except (ValueError, AttributeError):
                        continue
                if vertices:
                    poly = sg.Polygon(vertices)
                    if not poly.is_valid:
                        continue
                    # Subtract merged obstacles (if any) so that obstacles take priority.
                    if merged_obstacles is not None and (not merged_obstacles.is_empty):
                        new_poly = poly.difference(merged_obstacles)
                    else:
                        new_poly = poly
                    if new_poly.is_empty:
                        continue
                    # Retrieve cost (if needed; not used in output here).
                    try:
                        cost = float(widget.cost_input.text())
                    except (ValueError, AttributeError):
                        cost = 1.0

                    # Determine buffer resolution dynamically based on the original number of vertices.
                    resolution = max(1, round(len(vertices) / 4))
                    if new_poly.geom_type == "Polygon":
                        buffered_poly = new_poly.buffer(robot_radius, cap_style=2, resolution=resolution)
                        x_coords, y_coords = buffered_poly.exterior.xy
                        output += f"vector<Point> cost_zone_{cost_zone_index} = {{\n"
                        for x, y in zip(x_coords, y_coords):
                            if (output.find(f"  Point({x:.4f}, {y:.4f}),") == -1):
                                output += f"  Point({x:.4f}, {y:.4f}),\n"
                        output += "};\n\n"
                        cost_zone_index += 1
                    elif new_poly.geom_type == "MultiPolygon":
                        for poly_part in new_poly.geoms:
                            buffered_poly = poly_part.buffer(robot_radius, cap_style=2, resolution=resolution)
                            x_coords, y_coords = buffered_poly.exterior.xy
                            output += f"vector<Point> cost_zone_{cost_zone_index} = {{\n"
                            for x, y in zip(x_coords, y_coords):
                                if (output.find(f"  Point({x:.4f}, {y:.4f}),") == -1):
                                    output += f"  Point({x:.4f}, {y:.4f}),\n"
                            output += "};\n\n"
                            cost_zone_index += 1

        QtGui.QGuiApplication.clipboard().setText(output)
        QtWidgets.QMessageBox.information(
            self, "Copied",
            "Buffered vertices have been copied to clipboard."
        )

    def regenerate_plot(self):
        self.ax.clear()

        try:
            robot_radius = float(self.robot_radius.text())
        except ValueError:
            robot_radius = 0.0

        all_xs, all_ys = [], []
        obstacle_polygons = []
        cost_zone_data = []  # list of tuples (polygon, cost)

        # Collect polygons from all tabs.
        for tab_index in range(self.tabs.count()):
            widget = self.tabs.widget(tab_index)
            table = widget.table
            xs, ys = [], []
            for row in range(table.rowCount()):
                item_x = table.item(row, 0)
                item_y = table.item(row, 1)
                if item_x and item_y:
                    try:
                        x, y = float(item_x.text()), float(item_y.text())
                        xs.append(x)
                        ys.append(y)
                        all_xs.append(x)
                        all_ys.append(y)
                    except ValueError:
                        continue
            if xs and ys:
                polygon = sg.Polygon(list(zip(xs, ys)))
                if polygon.is_valid:
                    if self.tabs.tabToolTip(tab_index) == "Cost Zone":
                        try:
                            cost = float(widget.cost_input.text())
                        except (ValueError, AttributeError):
                            cost = 1.0
                        cost_zone_data.append((polygon, cost))
                    elif self.tabs.tabToolTip(tab_index) == "Obstacle":
                        obstacle_polygons.append(polygon)

        # Merge overlapping obstacles with a tolerance trick.
        if obstacle_polygons:
            tol = 0.001
            merged_obstacles = unary_union([p.buffer(tol) for p in obstacle_polygons]).buffer(-tol)
        else:
            merged_obstacles = None

        # Convert merged_obstacles to a list of polygons.
        if merged_obstacles is None or merged_obstacles.is_empty:
            merged_obstacles_list = []
        elif merged_obstacles.geom_type == "Polygon":
            merged_obstacles_list = [merged_obstacles]
        elif merged_obstacles.geom_type == "MultiPolygon":
            merged_obstacles_list = list(merged_obstacles.geoms)
        else:
            merged_obstacles_list = []

        # Process cost zones to subtract obstacles, preserving cost values.
        valid_cost_zone_data = []
        for cost_zone, cost in cost_zone_data:
            if merged_obstacles is None or merged_obstacles.is_empty:
                valid_cost_zone_data.append((cost_zone, cost))
            else:
                new_cost_zone = cost_zone.difference(merged_obstacles)
                if new_cost_zone.is_empty:
                    continue
                if new_cost_zone.geom_type == "Polygon":
                    valid_cost_zone_data.append((new_cost_zone, cost))
                elif new_cost_zone.geom_type == "MultiPolygon":
                    for poly in new_cost_zone.geoms:
                        valid_cost_zone_data.append((poly, cost))

        # Compute cost range for color scaling.
        if valid_cost_zone_data:
            costs = [c for (_, c) in valid_cost_zone_data]
            min_cost = min(costs)
            max_cost = max(costs)
        else:
            min_cost, max_cost = 0, 1

        # Plot valid cost zones with dynamic pigmentation (yellow to red).
        for poly, cost in valid_cost_zone_data:
            buffered_poly = poly.buffer(robot_radius, cap_style=2)
            x, y = buffered_poly.exterior.xy
            factor = (cost - min_cost) / (max_cost - min_cost) if max_cost != min_cost else 0
            color = (1.0, 1.0 - factor, 0.0)  # Yellow to red.
            self.ax.add_patch(plt.Polygon(list(zip(x, y)), closed=True,
                                          facecolor=color, edgecolor="green", alpha=0.5))

        # Plot obstacles.
        for poly in merged_obstacles_list:
            buffered_poly = poly.buffer(robot_radius, cap_style=2)
            x, y = buffered_poly.exterior.xy
            self.ax.add_patch(plt.Polygon(list(zip(x, y)), closed=True,
                                          facecolor="lightblue", alpha=1.0))

        # Set plot limits using provided field dimensions or auto-scale based on vertices.
        try:
            field_x, field_y = float(self.field_x.text()), float(self.field_y.text())
            self.ax.set_xlim(0, field_x)
            self.ax.set_ylim(0, field_y)
        except ValueError:
            self.ax.set_xlim(min(all_xs, default=0) - 1, max(all_xs, default=10) + 1)
            self.ax.set_ylim(min(all_ys, default=0) - 1, max(all_ys, default=10) + 1)

        self.ax.set_aspect("equal", "box")
        self.plot.draw()

    def export_map(self):
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export Map", "", "JSON Files (*.json)")
        if filename:
            data = {"obstacles": [], "cost_zones": []}
            for tab_index in range(self.tabs.count()):
                widget = self.tabs.widget(tab_index)
                table = widget.table
                vertices = []
                for row in range(table.rowCount()):
                    try:
                        x = float(table.item(row, 0).text())
                        y = float(table.item(row, 1).text())
                        vertices.append((x, y))
                    except (ValueError, AttributeError):
                        continue
                if self.tabs.tabToolTip(tab_index) == "Obstacle":
                    data["obstacles"].append(vertices)
                elif self.tabs.tabToolTip(tab_index) == "Cost Zone":
                    cost = 1.0
                    if hasattr(widget, "cost_input"):
                        try:
                            cost = float(widget.cost_input.text())
                        except ValueError:
                            cost = 1.0
                    data["cost_zones"].append({"vertices": vertices, "cost": cost})
            with open(filename, "w") as f:
                json.dump(data, f)

    def import_map(self):
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Import Map", "", "JSON Files (*.json)")
        if filename:
            with open(filename, "r") as f:
                data = json.load(f)
            # Clear existing tabs.
            self.tabs.clear()
            self.n_obstacles = 0
            self.n_cost_zones = 0
            for vertices in data.get("obstacles", []):
                polygon_widget = MyWidget(self.regenerate_plot)
                for x, y in vertices:
                    row = polygon_widget.table.rowCount()
                    polygon_widget.table.insertRow(row)
                    polygon_widget.table.setItem(row, 0, QtWidgets.QTableWidgetItem(str(x)))
                    polygon_widget.table.setItem(row, 1, QtWidgets.QTableWidgetItem(str(y)))
                self.tabs.addTab(polygon_widget, f"Obstacle {self.n_obstacles}")
                self.tabs.setTabToolTip(self.tabs.count() - 1, "Obstacle")
                self.n_obstacles += 1
            for zone_data in data.get("cost_zones", []):
                vertices, cost = zone_data.get("vertices", []), zone_data.get("cost", 1.0)
                cost_zone_widget = MyWidget(self.regenerate_plot)
                cost_zone_widget.cost_input = QtWidgets.QLineEdit(str(cost))
                cost_zone_widget.cost_input.setPlaceholderText("Cost")
                cost_zone_widget.layout.addWidget(cost_zone_widget.cost_input)
                cost_zone_widget.cost_input.textChanged.connect(self.regenerate_plot)
                for x, y in vertices:
                    row = cost_zone_widget.table.rowCount()
                    cost_zone_widget.table.insertRow(row)
                    cost_zone_widget.table.setItem(row, 0, QtWidgets.QTableWidgetItem(str(x)))
                    cost_zone_widget.table.setItem(row, 1, QtWidgets.QTableWidgetItem(str(y)))
                self.tabs.addTab(cost_zone_widget, f"Cost Zone {self.n_cost_zones}")
                self.tabs.setTabToolTip(self.tabs.count() - 1, "Cost Zone")
                self.n_cost_zones += 1
            self.regenerate_plot()


if __name__ == "__main__":
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
