from PySide6 import QtWidgets, QtCore, QtGui
import sys
import matplotlib
import json
import os.path
import shapely.geometry as sg

matplotlib.use('Qt5Agg')

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
import matplotlib.pyplot as plt
from matplotlib.figure import Figure

from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union


class MyWidget(QtWidgets.QWidget):
    def __init__(self, redraw_func=None):
        super().__init__()

        self.table = QtWidgets.QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["X", "Y"])
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.addWidget(self.table)

        self.add_row_button = QtWidgets.QPushButton("Add row")
        self.remove_row_button = QtWidgets.QPushButton("Remove row")
        self.layout.addWidget(self.add_row_button)
        self.layout.addWidget(self.remove_row_button)
        self.add_row_button.clicked.connect(self.add_row)
        self.remove_row_button.clicked.connect(self.remove_row)

        if redraw_func is not None:
            self.table.cellChanged.connect(redraw_func)
            self.remove_row_button.clicked.connect(redraw_func)

    def add_row(self):
        self.table.setRowCount(self.table.rowCount() + 1)

    def remove_row(self):
        self.table.setRowCount(self.table.rowCount() - 1)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Map Generator with Cost Zones")
        self.setGeometry(100, 100, 800, 600)

        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QtWidgets.QVBoxLayout(self.central_widget)

        # Tabs for obstacles and cost zones
        self.tabs = QtWidgets.QTabWidget()
        self.layout.addWidget(self.tabs)

        # Buttons
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

        self.make_polygon_button.clicked.connect(self.add_polygon)
        self.add_cost_zone_button.clicked.connect(self.add_cost_zone)
        self.remove_zone_button.clicked.connect(self.remove_zone)
        self.copy_vertices_button.clicked.connect(self.copy_vertices)
        self.export_map_button.clicked.connect(self.export_map)
        self.import_map_button.clicked.connect(self.import_map)

        # Input fields
        self.field_x = QtWidgets.QLineEdit()
        self.field_x.setPlaceholderText("Field X")
        self.field_y = QtWidgets.QLineEdit()
        self.field_y.setPlaceholderText("Field Y")
        self.robot_radius = QtWidgets.QLineEdit()
        self.robot_radius.setPlaceholderText("Robot Radius")
        self.layout.addWidget(self.field_x)
        self.layout.addWidget(self.field_y)
        self.layout.addWidget(self.robot_radius)

        self.field_x.setValidator(QtGui.QDoubleValidator())
        self.field_y.setValidator(QtGui.QDoubleValidator())
        self.robot_radius.setValidator(QtGui.QDoubleValidator())
        self.field_x.textEdited.connect(self.regenerate_plot)
        self.field_y.textEdited.connect(self.regenerate_plot)
        self.robot_radius.textEdited.connect(self.regenerate_plot)

        # Plotting area
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.plot = FigureCanvasQTAgg(self.fig)
        self.layout.addWidget(self.plot)

        # Storage
        self.n_obstacles = 0
        self.n_cost_zones = 0

    def add_polygon(self):
        polygon = MyWidget(self.regenerate_plot)
        self.tabs.addTab(polygon, f"Obstacle {self.n_obstacles}")
        self.tabs.setTabToolTip(self.tabs.count() - 1, "Obstacle")
        self.n_obstacles += 1
        self.regenerate_plot()

    def add_cost_zone(self):
        cost_zone_widget = MyWidget(self.regenerate_plot)
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
        output = "// Auto-generated list of buffered vertices\n\n"

        try:
            robot_radius = float(self.robot_radius.text())
        except ValueError:
            robot_radius = 0.0

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
                    except ValueError:
                        continue

            if xs and ys:
                polygon = sg.Polygon(list(zip(xs, ys)))

                if self.tabs.tabToolTip(tab_index) == "Obstacle":
                    if polygon.is_valid:
                        buffered_polygon = polygon.buffer(robot_radius, cap_style=2)
                        simplified_polygon = buffered_polygon.simplify(0.05, preserve_topology=True)
                        x_buff, y_buff = simplified_polygon.exterior.xy
                        output += f"vector<Point> obstacle_{tab_index} = {{\n"
                        for xb, yb in zip(x_buff, y_buff):
                            output += f"  Point({xb:.3f}, {yb:.3f}),\n"
                        output += "};\n\n"
                elif self.tabs.tabToolTip(tab_index) == "Cost Zone":
                    if polygon.is_valid:
                        buffered_polygon = polygon.buffer(robot_radius, cap_style=2)
                        simplified_polygon = buffered_polygon.simplify(0.05, preserve_topology=True)
                        x_buff, y_buff = simplified_polygon.exterior.xy
                        output += f"vector<Point> cost_zone_{tab_index} = {{\n"
                        for xb, yb in zip(x_buff, y_buff):
                            output += f"  Point({xb:.3f}, {yb:.3f}),\n"
                        output += "};\n\n"

        clipboard = QtGui.QGuiApplication.clipboard()
        clipboard.setText(output)
        QtWidgets.QMessageBox.information(self, "Copied", "Simplified buffered vertices have been copied to clipboard.")

    def regenerate_plot(self):
        self.ax.clear()

        try:
            robot_radius = float(self.robot_radius.text())
        except ValueError:
            robot_radius = 0.0

        all_xs, all_ys = [], []
        obstacle_polygons = []
        cost_zone_polygons = []

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
                        cost_zone_polygons.append(polygon)
                    elif self.tabs.tabToolTip(tab_index) == "Obstacle":
                        obstacle_polygons.append(polygon)

        # Merge overlapping obstacles
        merged_obstacles = unary_union(obstacle_polygons)

        # Create a new list for valid cost zones that do not overlap with obstacles
        valid_cost_zones = []
        for cost_zone in cost_zone_polygons:
            if merged_obstacles.is_empty:
                valid_cost_zones.append(cost_zone)  # No obstacles, keep the cost zone
            else:
                # Subtract the obstacles from the cost zone
                new_cost_zone = cost_zone.difference(merged_obstacles)
                if new_cost_zone.is_empty:
                    continue  # Skip if the resulting polygon is empty
                if isinstance(new_cost_zone, (Polygon, MultiPolygon)):
                    if isinstance(new_cost_zone, Polygon):
                        new_cost_zone = [new_cost_zone]
                    valid_cost_zones.extend(new_cost_zone)

        # Plot valid cost zones
        for poly in valid_cost_zones:
            buffered_polygon = poly.buffer(robot_radius, cap_style=2)
            x, y = buffered_polygon.exterior.xy
            hue = max(0.0, min(1.0, 0.5))  # Adjust cost hue scaling here
            color = (1.0, 1.0 - hue, 0.0)
            self.ax.add_patch(plt.Polygon(list(zip(x, y)), closed=True, facecolor=color, edgecolor="green", alpha=0.5))

        # Plot obstacles last with alpha set to 1.0 for opacity
        if isinstance(merged_obstacles, (Polygon, MultiPolygon)):
            if isinstance(merged_obstacles, Polygon):
                merged_obstacles = [merged_obstacles]
            for poly in merged_obstacles:
                buffered_polygon = poly.buffer(robot_radius, cap_style=2)
                x, y = buffered_polygon.exterior.xy
                self.ax.add_patch(plt.Polygon(list(zip(x, y)), closed=True, facecolor="purple", alpha=1.0))

        # Set plot limits
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
                vertices = [(float(table.item(row, 0).text()), float(table.item(row, 1).text()))
                            for row in range(table.rowCount())]

                if self.tabs.tabToolTip(tab_index) == "Obstacle":
                    data["obstacles"].append(vertices)
                elif self.tabs.tabToolTip(tab_index) == "Cost Zone":
                    cost = widget.cost_input.text()
                    try:
                        cost = float(cost)
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

            for vertices in data["obstacles"]:
                polygon = MyWidget(self.regenerate_plot)
                for x, y in vertices:
                    row = polygon.table.rowCount()
                    polygon.table.insertRow(row)
                    polygon.table.setItem(row, 0, QtWidgets.QTableWidgetItem(str(x)))
                    polygon.table.setItem(row, 1, QtWidgets.QTableWidgetItem(str(y)))
                self.tabs.addTab(polygon, f"Obstacle {self.tabs.count()}")
                self.tabs.setTabToolTip(self.tabs.count() - 1, "Obstacle")

            for zone_data in data["cost_zones"]:
                vertices, cost = zone_data["vertices"], zone_data["cost"]
                cost_zone_widget = MyWidget(self.regenerate_plot)
                cost_zone_widget.cost_input = QtWidgets.QLineEdit(str(cost))
                cost_zone_widget.layout.addWidget(cost_zone_widget.cost_input)
                for x, y in vertices:
                    row = cost_zone_widget.table.rowCount()
                    cost_zone_widget.table.insertRow(row)
                    cost_zone_widget.table.setItem(row, 0, QtWidgets.QTableWidgetItem(str(x)))
                    cost_zone_widget.table.setItem(row, 1, QtWidgets.QTableWidgetItem(str(y)))
                self.tabs.addTab(cost_zone_widget, f"Cost Zone {self.tabs.count()}")
                self.tabs.setTabToolTip(self.tabs.count() - 1, "Cost Zone")

            self.regenerate_plot()


if __name__ == "__main__":
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
