from pgmagick import Image, Color, Geometry, DrawableArc, DrawableLine, DrawableList, DrawableFillColor
from LogReader import LogReader
import pickle
import math


class LogAnalyzer:
    def __init__(self):
        self.rdrLog = LogReader()
        self.arrColors = ["white", "red", "blue", "yellow", "black"]

    def analyzeLog(self, strPath):
        log = self.rdrLog.loadLog(strPath)
        data = log.getOwlData()["task-tree"]

        #with open("data.pkl", "r") as f:
        #    pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)
        #    data = pickle.load(f)

        data = self.correctTime(data)

        imgTaskPie = Image(Geometry(700, 700), Color("white"))

        imgTaskPie.strokeColor("#000000")
        imgTaskPie.strokeWidth(2.5)
        imgTaskPie.fillColor("transparent")

        self.drawTaskPie(imgTaskPie, data, -1, -1, 5)
        imgTaskPie.write("out.png")

        toTasks = self.timelyOrderedTasks(data)

    def timelyOrderedTasks(self, data):
        dicLinear = self.linearizeTaskTree(data)
        arrItems = []

        for strItem in dicLinear:
            arrItems.append({"name": strItem,
                             "time": dicLinear[strItem]})

        return sorted(arrItems, key=lambda item: item["time"])

    def linearizeTaskTree(self, tree):
        dicLinear = {}

        for strBranch in tree:
            dicLinear[strBranch] = tree[strBranch]["time"]
            dicSub = self.linearizeTaskTree(tree[strBranch]["children"])
            dicLinear = dict(dicLinear, **dicSub)

        return dicLinear

    def correctTime(self, data):
        for strBranchName in data:
            data[strBranchName]["children"] = self.correctTime(data[strBranchName]["children"])

            nTimeSum = 0
            for strChild in data[strBranchName]["children"]:
                nTimeSum += data[strBranchName]["children"][strChild]["time"]

            if data[strBranchName]["time"] < nTimeSum:
                data[strBranchName]["time"] = nTimeSum

        return data

    def drawTaskPie(self, imgPie, dicTaskTree, globalTimespan = -1, parentTimespan = -1, radiusDelta = 10, radiusInner = 0, angleStart = 0, angleEnd = 360):
        if globalTimespan == -1:
            globalTimespan = 0
            for strBranchName in dicTaskTree:
                globalTimespan += dicTaskTree[strBranchName]["time"]

        if parentTimespan == -1:
            parentTimespan = 0
            for strBranchName in dicTaskTree:
                parentTimespan += dicTaskTree[strBranchName]["time"]

        if parentTimespan > 0:
            nSegments = len(dicTaskTree)

            radiusOuter = radiusInner + radiusDelta

            nCenterX = imgPie.columns() / 2
            nCenterY = imgPie.rows() / 2

            nStartXOuter = nCenterX - radiusOuter
            nStartYOuter = nCenterY - radiusOuter
            nEndXOuter = nCenterX + radiusOuter
            nEndYOuter = nCenterY + radiusOuter

            nStartXInner = nCenterX - radiusInner
            nStartYInner = nCenterY - radiusInner
            nEndXInner = nCenterX + radiusInner
            nEndYInner = nCenterY + radiusInner

            dAngleOffset = 0

            for strBranchName in dicTaskTree:
                dAngleWidth = float(dicTaskTree[strBranchName]["time"]) / float(parentTimespan) * (angleEnd - angleStart)

                if dAngleWidth > 0:
                    dStartingAngle = angleStart + dAngleOffset
                    dEndingAngle = dStartingAngle + dAngleWidth
                    dAngleOffset += dAngleWidth

                    if "children" in dicTaskTree[strBranchName]:
                        if len(dicTaskTree[strBranchName]["children"]) > 0:
                            self.drawTaskPie(imgPie, dicTaskTree[strBranchName]["children"], globalTimespan, dicTaskTree[strBranchName]["time"], radiusDelta, radiusOuter, dStartingAngle, dEndingAngle)

                    dTimeSpanDegree = float(dicTaskTree[strBranchName]["time"]) / float(globalTimespan)
                    imgPie.strokeColor(Color(int(255 * dTimeSpanDegree), 0, int(255 * (1.0 - dTimeSpanDegree))))

                    lstDrawables = DrawableList()
                    lstDrawables.append(DrawableLine(nCenterX + radiusInner * math.cos(math.radians(dStartingAngle)),
                                                     nCenterY + radiusInner * math.sin(math.radians(dStartingAngle)),
                                                     nCenterX + radiusOuter * math.cos(math.radians(dStartingAngle)),
                                                     nCenterY + radiusOuter * math.sin(math.radians(dStartingAngle))))
                    lstDrawables.append(DrawableArc(nStartXOuter, nStartYOuter, nEndXOuter, nEndYOuter, dStartingAngle, dEndingAngle))
                    lstDrawables.append(DrawableLine(nCenterX + radiusInner * math.cos(math.radians(dEndingAngle)),
                                                     nCenterY + radiusInner * math.sin(math.radians(dEndingAngle)),
                                                     nCenterX + radiusOuter * math.cos(math.radians(dEndingAngle)),
                                                     nCenterY + radiusOuter * math.sin(math.radians(dEndingAngle))))
                    lstDrawables.append(DrawableArc(nStartXInner, nStartYInner, nEndXInner, nEndYInner, dStartingAngle, dEndingAngle))

                    imgPie.draw(lstDrawables)