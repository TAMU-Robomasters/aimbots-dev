from PySide6 import QtWidgets, QtCore, QtGui
import sys
import matplotlib
import json
import os.path
import numpy as np
import shapely.geometry as sg

matplotlib.use('Qt5Agg')

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
import matplotlib.pyplot as plt
from matplotlib.figure import Figure

class MyWidget(QtWidgets.QWidget):
    def __init__(self, redraw_func = None):
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

        self.setWindowTitle("Map Generator")
        self.setGeometry(100, 100, 800, 600)
        
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QtWidgets.QVBoxLayout(self.central_widget)
        self.tabs = QtWidgets.QTabWidget()
        self.layout.addWidget(self.tabs)
        self.make_polygon_button = QtWidgets.QPushButton("Add obstacle")
        self.remove_polygon_button = QtWidgets.QPushButton("Remove obstacle")
        self.export_map_button = QtWidgets.QPushButton("Export Map")
        self.import_map_button = QtWidgets.QPushButton("Import Map")
        self.get_obstacles_button = QtWidgets.QPushButton("Get obstacles")
        self.layout.addWidget(self.make_polygon_button)
        self.layout.addWidget(self.remove_polygon_button)
        self.layout.addWidget(self.export_map_button)
        self.layout.addWidget(self.import_map_button)
        self.layout.addWidget(self.get_obstacles_button)
        self.make_polygon_button.clicked.connect(self.add_polygon)
        self.remove_polygon_button.clicked.connect(self.remove_polygon)
        self.export_map_button.clicked.connect(self.export_map)
        self.import_map_button.clicked.connect(self.import_map)
        self.get_obstacles_button.clicked.connect(self.export_obstacles)
        self.n = 0
        
        #text entry box for ballpark and robot radius
        self.field_x = QtWidgets.QLineEdit()
        self.field_x.setPlaceholderText("field x")
        self.field_y = QtWidgets.QLineEdit()
        self.field_y.setPlaceholderText("field y")
        self.robot_radius = QtWidgets.QLineEdit()
        self.robot_radius.setPlaceholderText("Robot radius")
        
        self.field_x.setValidator(QtGui.QDoubleValidator())
        self.field_y.setValidator(QtGui.QDoubleValidator())
        self.robot_radius.setValidator(QtGui.QDoubleValidator())
        
        self.field_x.textEdited.connect(self.regenerate_plot)
        self.field_y.textEdited.connect(self.regenerate_plot)
        self.robot_radius.textEdited.connect(self.regenerate_plot)
        self.layout.addWidget(self.field_x)
        self.layout.addWidget(self.field_y)
        self.layout.addWidget(self.robot_radius)

        
        self.tabs.currentChanged.connect(self.regenerate_plot)
        
        self.tabs.tabBar().tabBarDoubleClicked.connect(self.rename_tab)

        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.plot = FigureCanvasQTAgg(self.fig)
        self.layout.addWidget(self.plot)

    def add_polygon(self):
        polygon = MyWidget(self.regenerate_plot)
        self.tabs.addTab(polygon, "obstacle " + str(self.n))
        self.n += 1
        self.regenerate_plot()  # Regenerate plot after adding a new polygon
        
    def remove_polygon(self):
        current_index = self.tabs.currentIndex()
        if current_index != -1:
            self.tabs.removeTab(current_index)
            self.regenerate_plot()
    
    def regenerate_plot(self):
        self.ax.clear()
        
        all_xs = []
        all_ys = []
        

        for tab in range(self.tabs.count()):
            widget = self.tabs.widget(tab)
            table = widget.table
            xs = []
            ys = []
            for row in range(table.rowCount()):
                item_x = table.item(row, 0)
                item_y = table.item(row, 1)
                if item_x and item_y and item_x.text() and item_y.text():
                    try:
                        x = float(item_x.text())
                        y = float(item_y.text())
                        xs.append(x)
                        ys.append(y)
                        all_xs.append(x)
                        all_ys.append(y)
                    except ValueError:
                        print(f"Invalid data in obstacle {tab}, row {row}")

            if xs and ys:
                vertices = list(zip(xs, ys))
                radius = 0  # Default value for radius en
                try:
                    radius = float(self.robot_radius.text())
                except ValueError:
                    pass
                
                xse, yse = expand_polygon(vertices, radius)
                
                edge_color = 'black' if tab != self.tabs.currentIndex() else 'red'
                face_color = 'gray' if tab != self.tabs.currentIndex() else 'lightcoral'
                edge_color_expanded = 'blue' if tab != self.tabs.currentIndex() else 'lightblue'
                face_color_expanded = 'lightgray' if tab != self.tabs.currentIndex() else 'lightblue'
                

                self.ax.add_patch(plt.Polygon(list(zip(xs, ys)), closed=True, fill=True, edgecolor=edge_color, facecolor=face_color))
                self.ax.add_patch(plt.Polygon(list(zip(xse, yse)), closed=True, fill=True, edgecolor=edge_color_expanded, facecolor=face_color_expanded, alpha=0.3))

        try:
            field_x = float(self.field_x.text())
            field_y = float(self.field_y.text())
            self.ax.set_xlim(0, field_x)
            self.ax.set_ylim(0, field_y)
        except ValueError:
            if all_xs and all_ys:
                self.ax.set_xlim(min(all_xs) - 1, max(all_xs) + 1)
                self.ax.set_ylim(min(all_ys) - 1, max(all_ys) + 1)
            else:
                self.ax.set_xlim(0, 10)
                self.ax.set_ylim(0, 10)

        self.ax.set_aspect('equal', 'box')
        self.fig.tight_layout()
        self.plot.draw()
        
    def rename_tab(self, index):
        current_name = self.tabs.tabText(index)
        new_name, ok = QtWidgets.QInputDialog.getText(self, "Rename Tab", "New tab name:", 
                                            QtWidgets.QLineEdit.Normal, current_name)
        if ok and new_name:
            self.tabs.setTabText(index, new_name)
            
    def export_map(self):
        filename, ok = QtWidgets.QInputDialog.getText(self, "Export map", "Map name:", 
                                            QtWidgets.QLineEdit.Normal, "map.json")
        if not ok: return
        if not(filename):
            QtWidgets.QMessageBox.warning(self, "Error", "Something went wrong.")
            return
        
        with open(filename, 'w') as f:
            
            data = {}
            config = {
                "field_x": self.field_x.text(),
                "field_y": self.field_y.text(),
                "robot_radius": self.robot_radius.text()
            }
            data["config"] = config          
            data["obstacles"] = {}
            for tab in range(self.tabs.count()):
                widget = self.tabs.widget(tab)
                table = widget.table
                points = []
                for row in range(table.rowCount()):
                    item_x = table.item(row, 0)
                    item_y = table.item(row, 1)
                    if item_x and item_y and item_x.text() and item_y.text():
                        try:
                            x = float(item_x.text())
                            y = float(item_y.text())
                            points.append((x, y))
                        except ValueError:
                            print(f"Invalid data in obstacle {tab}, row {row}")
                data["obstacles"][self.tabs.tabText(tab)] = points
            json.dump(data, f)
        pass
    
    def import_map(self):
        filename, ok = QtWidgets.QInputDialog.getText(self, "Import map", "Map name:", 
                                            QtWidgets.QLineEdit.Normal, "map.json")
        
        if not ok: return
        if not (filename and os.path.isfile(filename)):
            # prompt user to select a valid file
            QtWidgets.QMessageBox.warning(self, "Error", "Invalid file selected.")
            return
        with open(filename, 'r') as f:
            data = json.load(f)
            config = data.get("config", {})
            self.field_x.setText(config.get("field_x", ""))
            self.field_y.setText(config.get("field_y", ""))
            self.robot_radius.setText(config.get("robot_radius", ""))
            
            # Extract obstacle data
            obstacles = data.get("obstacles", {})
            
            for name, points in obstacles.items():
                polygon = MyWidget(self.regenerate_plot)
                self.tabs.addTab(polygon, name)
                for x, y in points:
                    row_position = polygon.table.rowCount()
                    polygon.table.insertRow(row_position)
                    polygon.table.setItem(row_position, 0, QtWidgets.QTableWidgetItem(str(x)))
                    polygon.table.setItem(row_position, 1, QtWidgets.QTableWidgetItem(str(y)))
            self.regenerate_plot()
    
    def export_obstacles(self):
        output = "// Auto-generated list of obstacles using map generation utility\n\n"
        for tab in range(self.tabs.count()):
            widget = self.tabs.widget(tab)
            table = widget.table
            vertices = []
            for row in range(table.rowCount()):
                item_x = table.item(row, 0)
                item_y = table.item(row, 1)
                if item_x and item_y and item_x.text() and item_y.text():
                    vertices.append((float(item_x.text()), float(item_y.text())))

            # Expand the polygon
            radius = 0 
            try:
                radius = float(self.robot_radius.text())
            except ValueError:
                pass
            x, y = expand_polygon(vertices, radius)  # Adjust the distance as needed

            if x and y:
                output += f"vector<Point> {self.tabs.tabText(tab).replace(' ', '_')} = {{\n"
                for i in range(len(x)):
                    output += f"  Point({x[i]}, {y[i]}),\n"
                output += "};\n\n"
            else:
                output += f"// Could not expand polygon for {self.tabs.tabText(tab).replace(' ', '_')}\n\n"

        clipboard = QtGui.QGuiApplication.clipboard()
        clipboard.setText(output)
        QtWidgets.QMessageBox.information(self, "", "Obstacles copied to clipboard")
        
def expand_polygon(vertices, distance):
    polygon = sg.Polygon(vertices)
    expanded_polygon = polygon.buffer(distance, cap_style=2, join_style=2, mitre_limit=1)
    return expanded_polygon.exterior.coords.xy
    


if __name__ == "__main__":
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()    

    sys.exit(app.exec())