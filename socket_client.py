import socket, struct
import numpy as np
import cv2
import time

import vtk
import vtk.util.numpy_support as vtk_np

from config.config import *

# host = "52.237.12.175"
# port = 60000

connection = None
TYPE = 1
vtk_actor = None

class SocketClient:
    def __init__(self, host, port=60000):
        self.s = socket.socket()
        self.host = host
        self.port = port
        self.status = False
        try:
            self.s.connect((host, port))
            print("Connected to server", host, "port", port)

            t = struct.pack("I", TYPE)
            q = struct.pack("I", QUALITY_HIGH)
            self.s.send(t + q)

            # self.send(TYPE)
            # self.send(QUALITY_HIGH)
            self.status = True

        except ConnectionRefusedError as e:
            print(str(e))
            self.status = False

    def send(self, data):
        print("sending", struct.pack("I", int(data)))
        self.s.send(struct.pack("I", int(data)))

    def receive_with_length(self, length):
        received_data = self.s.recv(length)
        return received_data

    def disconnect(self):
        self.s.close()
        self.status = False
        print("Disconnect from server", self.host, "port", self.port)

    def __del__(self):
        self.disconnect()


class VTKActorWrapper(object):
    def __init__(self, np_array, colour_array=[], use_colour_points=False, require_transform=False, t_x=0, t_y=0,
                 t_z=0):
        super(VTKActorWrapper, self).__init__()

        self.np_array = np_array

        n_coords = np_array.shape[0]
        n_elem = np_array.shape[1]

        self.verts = vtk.vtkPoints()
        self.cells = vtk.vtkCellArray()
        self.scalars = None

        self.pd = vtk.vtkPolyData()
        self.verts.SetData(vtk_np.numpy_to_vtk(np_array))
        self.cells_npy = np.vstack([np.ones(n_coords, dtype=np.int64),
                                    np.arange(n_coords, dtype=np.int64)]).T.flatten()
        self.cells.SetCells(n_coords, vtk_np.numpy_to_vtkIdTypeArray(self.cells_npy))
        self.pd.SetPoints(self.verts)
        self.pd.SetVerts(self.cells)
        if use_colour_points:
            self.pd.GetPointData().SetScalars(colour_array)
        self.mapper = vtk.vtkPolyDataMapper()

        if require_transform:
            transform = vtk.vtkTransform()
            transform.RotateX(t_x)
            transform.RotateY(t_y)
            transform.RotateZ(t_z)
            transform_filter = vtk.vtkTransformPolyDataFilter()
            transform_filter.SetTransform(transform)
            transform_filter.SetInputDataObject(self.pd)
            transform_filter.Update()
            self.mapper.SetInputConnection(transform_filter.GetOutputPort())
        else:
            self.mapper.SetInputDataObject(self.pd)
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)
        self.actor.GetProperty().SetRepresentationToPoints()
        self.actor.GetProperty().SetColor(0.0, 1.0, 0.0)

    def update_actor(self,img):
        # while (update_on.is_set()):
        #     time.sleep(0.01)
        #     threadLock.acquire()
        #     cam.wait_for_frames()
        self.np_array[:] = img
        self.pd.Modified()
        # threadLock.release()


class PointCloud(object):
    def __init__(self, depth):
        super(PointCloud, self).__init__()

        """Transform a depth image into a point cloud with one point for each
        pixel in the image, using the camera transform for a camera
        centred at cx, cy with field of view fx, fy.

        depth is a 2-D ndarray with shape (rows, cols) containing
        depths from 1 to 254 inclusive. The result is a 3-D array with
        shape (rows, cols, 3). Pixels with invalid depth in the input have
        NaN for the z-coordinate in the result.

        """
        self.cx = 640 / 2
        self.cy = 320 / 2

        self.fx = 640 * 65.5 / 91.2
        self.fy = 320 * 91.2 / 65.5

        rows, cols = depth.shape

        c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
        self.z = depth / float(2 ** 8)
        valid = (self.z > 0) & (self.z < 218)
        self.z = np.where(valid, depth, np.nan)
        self.x = np.where(valid, self.z * (c - self.cx) / self.fx, 0)
        self.y = np.where(valid, self.z * (r - self.cy) / self.fy, 0)

        self.result = np.dstack((self.x, self.y, self.z))


class VTKVisualisation(object):
    def __init__(self, actor, axis=True, ):
        super(VTKVisualisation, self).__init__()

        self.ren = vtk.vtkRenderer()
        self.ren.AddActor(actor)

        self.axesActor = vtk.vtkAxesActor()
        self.axesActor.AxisLabelsOff()
        self.axesActor.SetTotalLength(1, 1, 1)
        self.ren.AddActor(self.axesActor)

        self.renWin = vtk.vtkRenderWindow()
        self.renWin.AddRenderer(self.ren)

        ## IREN
        self.iren = vtk.vtkRenderWindowInteractor()
        self.iren.SetRenderWindow(self.renWin)
        self.iren.Initialize()

        self.style = vtk.vtkInteractorStyleTrackballCamera()
        self.iren.SetInteractorStyle(self.style)

        self.iren.AddObserver("TimerEvent", self.update_visualisation)
        dt = 30  # ms
        timer_id = self.iren.CreateRepeatingTimer(dt)

        # self.iren.Start()

    def update_visualisation(self, obj=None, event=None):
        time.sleep(0.01)
        # self.threadLock.acquire()
        self.ren.GetRenderWindow().Render()
        # self.threadLock.release()

class SingleImage:
    def __init__(self):
        self.h = struct.unpack("I", connection.receive_with_length(4))[0]
        self.w = struct.unpack("I", connection.receive_with_length(4))[0]
        self.size = struct.unpack("I", connection.receive_with_length(4))[0]
        self.raw_img = None
        self.img = None
        self.point_cloud = None
    # def receive_header(self):
    #     h = c
    #     w = connection.receive_with_length(4)
    #     size = connection.receive_with_length(4)
    #     if h and w and size:
    #         self.h =
    #         self.w = struct.unpack("I", w)[0]
    #         self.size = struct.unpack("I", size)[0]
    #     # else:
    #         # raise StreamingEnd("Header not received")

    def receive_img(self):
        b_data = b''
        try:
            while len(b_data) < self.size:
                print(len(b_data), self.size)
                if len(b_data) + BUFFER_SIZE < self.size:
                    b_data += connection.receive_with_length(BUFFER_SIZE)
                else:
                    b_data += connection.receive_with_length(self.size - len(b_data))

            self.raw_img = np.asarray(list(bytearray(b_data)))
            # print(self.h, self.w, self.size, self.raw_img.size)

        except ConnectionError as e:
            print(str(e))
            connection.status = False
            connection.disconnect()

    def construct_img(self):
        # TODO This needs to be implement later, based on how we encode/compress the image
        self.img = np.reshape(self.raw_img, (self.h, self.w))
        self.img = self.img.astype(np.uint8)

    def construct_point_cloud(self):
        self.point_cloud = PointCloud(self.raw_img)
        # npy_z = vtk_np.numpy_to_vtk(point_cloud.z)

        # look_up_table = vtk.vtkLookupTable()
        # look_up_table.SetTableRange(signal_str.min(), signal_str.max())
        # look_up_table.SetTableRange(0, 1)
        # look_up_table.Build()

        # array = look_up_table.MapScalars(npy_z, vtk.VTK_COLOR_MODE_DEFAULT, -1)

    def show_img(self):
        cv2.imshow('img', self.img)
        cv2.waitKey(30)


if __name__ == "__main__":
    npy_depth1 = np.load('1520468815.npy')
    npy_depth2 = np.load('1520468813.npy')
    a = 1
    connection = SocketClient(host, port)
    connection.s.settimeout(1)
    # TODO Add GUI for change view options (3d RGB) and quality
    # TODO Create renderer, render window, etc for VTK
    # TODO 1.transform image to point cloud 2.vtk render
    while connection.status:
        single_image = None
        try:
            single_image = SingleImage()
            single_image.receive_img()
            single_image.construct_img()
            if vtk_actor is None:
                single_image.point_cloud = PointCloud(npy_depth1)
                vtk_actor = VTKActorWrapper(single_image.point_cloud.result.reshape(-1, 3), require_transform=True, t_x=180)
                viz = VTKVisualisation(vtk_actor.actor)
            else:
                if a:
                    single_image.point_cloud = PointCloud(npy_depth1)
                    a = 0
                else:
                    single_image.point_cloud = PointCloud(npy_depth2)
                    a = 1
                vtk_actor.update_actor(single_image.point_cloud.result.reshape(-1, 3))
            single_image.show_img()
        except socket.timeout as e:
            print("Streaming not started")
            cv2.destroyAllWindows()
            del single_image
            continue
        except ValueError as e:
            print(str(e))
            cv2.destroyAllWindows()
            del single_image
            continue
        except ConnectionError as e:
            print(str(e))
            connection.status = False
            connection.disconnect()

    print('Client Disconnected')
