/*/
MIT License

Copyright (c) 2017 hsimyu

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
//*/

#ifndef SIMPLE_VTK_HPP_INCLUDED
#define SIMPLE_VTK_HPP_INCLUDED

#include <iostream>
#include <cmath>
#include <array>
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <sstream>

#ifdef USE_BOOST
#include <functional>
#include <boost/array.hpp>
#include <boost/multi_array.hpp>
#endif

class SimpleVTK {
    private:
        const std::string header = R"(<?xml version="1.0" ?>
)";
        std::string content;
        std::string buffer;
        std::string newLine;
        bool endEdit = false;
        unsigned int innerElementPerLine = 10;
        std::string current_vtk_type;
        const std::string current_vtkHierarchicalBoxDataSet_version = "1.1";

        // indent management
        std::string indent;
        int currentIndentation = -1;
        void addIndent() {
            ++currentIndentation;
        }

        void backIndent() {
            if (currentIndentation >= 0) --currentIndentation;
        }

        void insertIndent() {
            for(unsigned int i = 0; i < currentIndentation; ++i) {
                buffer += indent;
            }
        }


        // store current processing tag information for type validation
        std::string current_tag;

        void setCurrentTag(const std::string& tag) {
            current_tag = tag;
        }

        std::string getCurrentTag() {
            return current_tag;
        }

        void beginWithHeader() {
            content += header;
        }

        // Structure Management
        void beginElement(const std::string& tag) {
            if (buffer != "") {
                commitBuffer();
            }

            addIndent();
            insertIndent();
            setCurrentTag(tag);

            buffer += "<" + getCurrentTag();
        }

        void endElement(const std::string& tag) {
            if (buffer != "") {
                commitBuffer();
            }

            insertIndent();
            buffer += "</" + tag;
            commitBuffer();

            backIndent();
        }

        void commitBuffer() {
            content += buffer + ">" + newLine;
            buffer.clear();
        }

        void endOneLineElement(const std::string& tag) {
            if (buffer != "") {
                commitOneLineBuffer();
            }
            backIndent();
        }

        void commitOneLineBuffer() {
            content += buffer + "/>" + newLine;
            buffer.clear();
        }

        void beginInnerElement() {
            if (buffer != "") {
                commitBuffer();
            }
            addIndent();
            insertIndent();
        }

        void endInnerElement() {
            if (buffer != "") {
                commitInnerBuffer();
            }
            backIndent();
        }

        void commitInnerBuffer() {
            content += buffer;
            buffer.clear();
        }


        // Type checking
        static constexpr int VTKFile_type_length = 12;
        std::array<std::string, VTKFile_type_length> VTKFileType {{
            "ImageData", "RectlinearGrid", "StructuredGrid", "PolyData", "UnstructuredGrid", "vtkHierarchicalBoxDataSet",
            "PImageData", "PRectlinearGrid", "PStructuredGrid", "PPolyData", "PUnstructuredGrid", "PvtkHierarchicalBoxDataSet",
        }};

        static constexpr int format_type_length = 3;
        std::array<std::string, format_type_length> FormatType {{"ascii", "binary", "appended"}};

        static constexpr int byte_order_type_length = 2;
        std::array<std::string, byte_order_type_length> ByteOrderType {{"LittleEndian", "BigEndian"}};

        static constexpr int number_type_length = 10;
        std::array<std::string, number_type_length> NumberType {{
            "Float32", "Float64",
            "Int8", "Int16", "Int32", "Int64",
            "UInt8", "UInt16", "UInt32", "UInt64"
        }};

        static constexpr int encoding_type_length = 2;
        std::array<std::string, encoding_type_length> EncodingType {{"base64", "raw"}};

        template<int N>
        bool checkIsValidType(const std::array<std::string, N>& valid_types, const std::string& specified_type) {
            bool is_valid = false;

            for(const auto& t : valid_types) {
                if (t == specified_type) {
                    is_valid = true;
                }
            }

            return is_valid;
        }

        bool checkType(const std::string& type) {
            bool isValid = false;

            if (current_tag == "VTKFile") {
                isValid = checkIsValidType<VTKFile_type_length>(VTKFileType, type);
            } else if (current_tag == "DataArray" || current_tag == "PDataArray") {
                isValid = checkIsValidType<number_type_length>(NumberType, type);
            } else {
                isValid = true;
            }

            if (!isValid) {
                std::string error_message = "[SIMPLE VTK ERROR] Invalid " + current_tag + " type = " + type + " is passed to.";
                throw std::invalid_argument(error_message);
            }

            return isValid;
        }

        std::string getConventionalFileExtensionFromCurrentVTKType() const {
            if (current_vtk_type == "ImageData") {
                return ".vti";
            } else if (current_vtk_type == "RectlinearGrid") {
                return ".vtr";
            } else if (current_vtk_type == "StructuredGrid") {
                return ".vts";
            } else if (current_vtk_type == "PolyData") {
                return ".vtp";
            } else if (current_vtk_type == "UnstructuredGrid") {
                return ".vtu";
            } else if (current_vtk_type == "vtkHierarchicalBoxDataSet") {
                return ".vthb";
            } else if (current_vtk_type == "PImageData") {
                return ".pvti";
            } else if (current_vtk_type == "PRectlinearGrid") {
                return ".pvtr";
            } else if (current_vtk_type == "PStructuredGrid") {
                return ".pvts";
            } else if (current_vtk_type == "PPolyData") {
                return ".pvtp";
            } else if (current_vtk_type == "PUnstructuredGrid") {
                return ".pvtu";
            } else if (current_vtk_type == "PvtkHierarchicalBoxDataSet") {
                return ".pvthb";
            } else {
                return ".unknown";
            }
        }

        // Adding Valid Attributes
        void addType(const std::string& type) {
            if (type == "") return;

            if (checkType(type)) {
                buffer += " type=\"" + type + "\"";

                if (current_tag == "VTKFile") {
                    current_vtk_type = type;
                }
            }
        }

        // for convinience, set attributes from variadic arguments
        void convertFromVariadicArgsToStringInternal(const std::string& buffer) {}

        template<typename First, typename... Rests>
        void convertFromVariadicArgsToStringInternal(std::string& buffer, First&& first, Rests&&... rests) {
            std::stringstream ss;
            ss << first;
            buffer = buffer + ss.str();

            constexpr std::size_t parameter_pack_size = sizeof...(Rests);
            if (parameter_pack_size > 0) {
                buffer = buffer + " ";
                convertFromVariadicArgsToStringInternal(buffer, std::forward<Rests>(rests)...);
            }
        }

        template<typename... Args>
        std::string convertFromVariadicArgsToString(Args&&... args) {
            std::string buffer = "";
            convertFromVariadicArgsToStringInternal(buffer, std::forward<Args>(args)...);
            return buffer;
        }

        void addItemInternal(std::stringstream& ss) {}

        template <typename First, typename... Rests>
        void addItemInternal(std::stringstream &ss, First &&first, Rests &&... rests) {
            ss << first;

            constexpr std::size_t parameter_pack_size = sizeof...(Rests);
            if (parameter_pack_size > 0) {
                ss << " ";
                addItemInternal(ss, std::forward<Rests>(rests)...);
            }
        }

        // Base Extent Management
        struct ExtentInfo_t {
            int xmin = 0;
            int xmax = 0;
            int ymin = 0;
            int ymax = 0;
            int zmin = 0;
            int zmax = 0;
            double x0 = 0.0;
            double y0 = 0.0;
            double z0 = 0.0;
            double dx = 0.0;
            double dy = 0.0;
            double dz = 0.0;
        };

        struct ExtentManagementStatus {
            bool extent_initialized = false;
            bool origin_initialized = false;
            bool spacing_initialized = false;
            bool enable = false;
        };

        ExtentInfo_t base_extent;
        ExtentManagementStatus manage_extent_status;

        // for AMR index management types
        struct AMRBox {
            int xmin;
            int xmax;
            int ymin;
            int ymax;
            int zmin;
            int zmax;
        };

        struct AMRBoxBlock {
            std::map<int, AMRBox> boxes;
        };

        struct AMRInfo_t {
            double refinement_ratio = 2.0;
            int current_level = 0;
            int current_index = 0;
            std::map<int, AMRBoxBlock> blocks;
        };

        AMRInfo_t amr_info;

        void addNewAMRBox(const int xmin, const int xmax, const int ymin, const int ymax, const int zmin, const int zmax) {
            if (amr_info.blocks.count(amr_info.current_level) == 0) {
                AMRBoxBlock new_block;
                amr_info.blocks[amr_info.current_level] = std::move(new_block);
            }

            auto& boxes = amr_info.blocks[amr_info.current_level].boxes;

            if (boxes.count(amr_info.current_index) == 0) {
                AMRBox new_box{xmin, xmax, ymin, ymax, zmin, zmax};
                boxes[amr_info.current_index] = std::move(new_box);
            }
        }

        void initializeAMRBoxInfo() {
            amr_info.current_level = 0;
            amr_info.current_index = 0;
            amr_info.blocks.clear();
        }

    public:
        SimpleVTK() {
            init();
        }

        // Initialize Current State
        void init() {
            setNewLineCodeLF();
            setIndentSpaceSize();
            endEdit = false;
            content = "";
            current_tag = "VTKFile";
            current_vtk_type = "";
            initializeAMRBoxInfo();
        }

        void setIndentSpaceSize(const int size = 4) {
            if (size == 0) {
                indent = '\t';
                return;
            }

            indent.clear();
            for (int i = 0; i < size; ++i) {
                indent += ' ';
            }
        }

        void setNewLineCodeLF() {
            constexpr char LF = '\n';
            newLine = LF;
        }

        void setNewLineCodeCR() {
            constexpr char CR = '\r';
            newLine = CR;
        }

        void setNewLineCodeCRLF() {
            constexpr const char* CRLF = "\r\n";
            newLine = CRLF;
        }

        void setInnerElementPerLine(unsigned int value) {
            innerElementPerLine = value;
        }

        // IO functions
        void generate(const std::string file_name) {
            if(!endEdit) endVTK();

            std::ofstream ofs(file_name + getConventionalFileExtensionFromCurrentVTKType(), std::ios::out);
            ofs << content;
        }

        std::string getRawString() const {
            return content;
        }

        void beginVTK(const std::string& type) {
            beginWithHeader();
            beginElement("VTKFile");
            addType(type);

            if (type == "vtkHierarchicalBoxDataSet") {
                setVersion(current_vtkHierarchicalBoxDataSet_version);
            }
        }

        void endVTK() {
            endElement("VTKFile");
            endEdit = true;
        }

        //! Do nothing except for type checking and setting
        void beginVTKPartial(const std::string& type) {
            if (checkType(type)) {
                current_vtk_type = type;
            }
        }

        void endVTKPartial() {
            endEdit = true;
        }

        void beginContent() {
            beginElement(current_vtk_type);

            if (isExtentManagementEnable()) {
                if (current_vtk_type != "vtkHierarchicalBoxDataSet") {
                    setWholeExtent(base_extent.xmin, base_extent.xmax, base_extent.ymin, base_extent.ymax, base_extent.zmin, base_extent.zmax);
                }

                if (current_vtk_type == "ImageData" || current_vtk_type == "PImageData") {
                    setOrigin(base_extent.x0, base_extent.y0, base_extent.z0);
                    setSpacing(base_extent.dx, base_extent.dy, base_extent.dz);
                } else if (current_vtk_type == "vtkHierarchicalBoxDataSet") {
                    setOrigin(base_extent.x0, base_extent.y0, base_extent.z0);
                }
            }
        }
        void endContent() { endElement(current_vtk_type); }

        void beginPiece() {
            beginElement("Piece");

            if (isExtentManagementEnable()) {
                setExtent(base_extent.xmin, base_extent.xmax, base_extent.ymin, base_extent.ymax, base_extent.zmin, base_extent.zmax);
            }
        }
        void endPiece() { endElement("Piece"); }

        void beginContentWithPiece() {
            if (isExtentManagementEnable()) {
                beginContent();
                beginPiece();
            } else {
                throw std::logic_error("[SIMPLE VTK ERROR] beginContentWithPiece() was called without initializing BaseExtent. Call changeBaseExtent(), changeBaseOrigin(), and changeBaseSpacing() before calling this function.");
            }
        }

        void endContentWithPiece() {
            if (isExtentManagementEnable()) {
                endPiece();
                endContent();
            } else {
                throw std::logic_error("[SIMPLE VTK ERROR] endContentWithPiece() was called without initializing BaseExtent. Call changeBaseExtent(), changeBaseOrigin(), and changeBaseSpacing() before calling this function.");
            }
        }

        void beginPointData() { beginElement("PointData"); }
        void endPointData() { endElement("PointData"); }
        void addPointData() {
            beginPointData();
            endPointData();
        }

        void beginCellData() { beginElement("CellData"); }
        void endCellData() { endElement("CellData"); }
        void addCellData() {
            beginCellData();
            endCellData();
        }

        void beginPoints() { beginElement("Points"); }
        void endPoints() { endElement("Points"); }
        void addPoints() {
            beginPoints();
            endPoints();
        }

        void beginCells() { beginElement("Cells"); }
        void endCells() { endElement("Cells"); }
        void addCells() {
            beginCells();
            endCells();
        }

        void beginBlock() {
            beginElement("Block");
            setLevel(amr_info.current_level);

            if (isExtentManagementEnable()) {
                const auto per_level = 1.0 / std::pow(amr_info.refinement_ratio, amr_info.current_level);
                setSpacing(base_extent.dx * per_level, base_extent.dy * per_level, base_extent.dz * per_level);
            }
        }

        void beginBlock(const int level) {
            beginElement("Block");
            setLevel(level);

            if (amr_info.current_level != level) {
                amr_info.current_level = level;
            }

            if (isExtentManagementEnable()) {
                const auto per_level = 1.0 / std::pow(amr_info.refinement_ratio, level);
                setSpacing(base_extent.dx * per_level, base_extent.dy * per_level, base_extent.dz * per_level);
            }
        }

        void beginBlock(const std::string level) {
            beginBlock(std::stoi(level));
        }

        void endBlock() {
            endElement("Block");
            amr_info.current_level++;
        }

        template<typename T>
        void beginDataSet(const T index) {
            beginElement("DataSet");
            setIndex(index);
        }
        void endDataSet() { endElement("DataSet"); }

        void beginAppendData() { beginElement("AppendData"); }
        void endAppendData() { endElement("AppendData"); }

        void beginDataArray(const std::string name, const std::string number_type, const std::string format) {
            beginElement("DataArray");
            setName(name);
            setNumberType(number_type);
            setFormat(format);
        }
        void endDataArray() { endElement("DataArray"); }

        // --- Attirbute Setting Functions ---
        void setName(const std::string& name) {
            if (name != "") {
                buffer += " Name=\"" + name + "\"";
            }
        }

        void setVersion(const std::string& _version) {
            buffer += " version=\"" + _version + "\"";
        }

        void setByteOrder(const std::string& byte_order) {
            if (checkIsValidType<byte_order_type_length>(ByteOrderType, byte_order)) {
                buffer += " byte_order=\"" + byte_order + "\"";
            } else {
                std::string error_message = "[SIMPLE VTK ERROR] Invalid ByteOrder type = " + byte_order + " is passed to setByteOrder().";
                throw std::invalid_argument(error_message);
            }
        }

        void setCompressor(const std::string& compressor) {
            buffer += " compressor=\"" + compressor + "\"";
        }

        void setFormat(const std::string& type) {
            if (checkIsValidType<format_type_length>(FormatType, type)) {
                buffer += " format=\"" + type + "\"";
            } else {
                std::string error_message = "[SIMPLE VTK ERROR] Invalid Format type = " + type + " is passed to setFormat().";
                throw std::invalid_argument(error_message);
            }
        }

        void setNumberType(const std::string& type) {
            if (checkIsValidType<number_type_length>(NumberType, type)) {
                buffer += " type=\"" + type + "\"";
            } else {
                std::string error_message = "[SIMPLE VTK ERROR] Invalid Number type = " + type + " is passed to setType().";
                throw std::invalid_argument(error_message);
            }
        }

        void setEncoding(const std::string& encoding) {
            if (checkIsValidType<encoding_type_length>(EncodingType, encoding)) {
                buffer += " encoding=\"" + encoding + "\"";
            } else {
                std::string error_message = "[SIMPLE VTK ERROR] Invalid Encoding type = " + encoding + " is passed to setEncoding().";
                throw std::invalid_argument(error_message);
            }
        }

        void setNumberOfPoints(const std::string& num) {
            buffer += " NumberOfPoints=\"" + num + "\"";
        }

        void setNumberOfCells(const std::string& num) {
            buffer += " NumberOfCells=\"" + num + "\"";
        }

        void setNumberOfComponents(const std::string& num) {
            buffer += " NumberOfComponents=\"" + num + "\"";
        }

        void setNumberOfPoints(const int num) {
            buffer += " NumberOfPoints=\"" + std::to_string(num) + "\"";
        }

        void setNumberOfCells(const int num) {
            buffer += " NumberOfCells=\"" + std::to_string(num) + "\"";
        }

        void setNumberOfComponents(const int num) {
            buffer += " NumberOfComponents=\"" + std::to_string(num) + "\"";
        }

        void setGridDescription(const std::string& desc) {
            buffer += " grid_description=\"" + desc + "\"";
        }

        void setLevel(const std::string& level) {
            buffer += " level=\"" + level + "\"";
        }

        void setLevel(const int level) {
            buffer += " level=\"" + std::to_string(level) + "\"";
        }

        void setIndex(const std::string& index) {
            amr_info.current_index = std::stoi(index);
            buffer += " index=\"" + index + "\"";
        }

        void setIndex(const int index) {
            amr_info.current_index = index;
            buffer += " index=\"" + std::to_string(index) + "\"";
        }

        void setFile(const std::string file_path) {
            buffer += " file=\"" + file_path + "\"";
        }

        void setOffset(const std::string& num) {
            buffer += " offset=\"" + num + "\"";
        }

        //! For PointData and CellData
        template<typename... Args>
        void setScalars(Args&&... args) {
            std::string scalars = convertFromVariadicArgsToString(std::forward<Args>(args)...);
            buffer += " Scalars=\"" + scalars + "\"";
        }

        template<typename... Args>
        void setVectors(Args&&... args) {
            std::string vectors = convertFromVariadicArgsToString(std::forward<Args>(args)...);
            buffer += " Vectors=\"" + vectors + "\"";
        }

        template<typename... Args>
        void setTensors(Args&&... args) {
            std::string tensors = convertFromVariadicArgsToString(std::forward<Args>(args)...);
            buffer += " Tensors=\"" + tensors + "\"";
        }

        template<typename... Args>
        void setTCoords(Args&&... args) {
            std::string tcoords = convertFromVariadicArgsToString(std::forward<Args>(args)...);
            buffer += " TCoords=\"" + tcoords + "\"";
        }

        template<typename... Args>
        void setWholeExtent(Args&&... args) {
            std::string whole_extent = convertFromVariadicArgsToString(std::forward<Args>(args)...);
            buffer += " WholeExtent=\"" + whole_extent + "\"";
        }

        template<typename... Args>
        void setExtent(Args&&... args) {
            std::string extent = convertFromVariadicArgsToString(std::forward<Args>(args)...);
            buffer += " Extent=\"" + extent + "\"";
        }

        template<typename... Args>
        void setOrigin(Args&&... args) {
            std::string origin = convertFromVariadicArgsToString(std::forward<Args>(args)...);

            if (current_vtk_type != "vtkHierarchicalBoxDataSet") {
                buffer += " Origin=\"" + origin + "\"";
            } else {
                buffer += " origin=\"" + origin + "\""; // ;D
            }
        }

        template<typename... Args>
        void setSpacing(Args&&... args) {
            std::string spacing = convertFromVariadicArgsToString(std::forward<Args>(args)...);
            if (current_vtk_type != "vtkHierarchicalBoxDataSet") {
                buffer += " Spacing=\"" + spacing + "\"";
            } else {
                buffer += " spacing=\"" + spacing + "\""; // ;D
            }
        }

        void setAMRBox(const int xmin, const int xmax, const int ymin, const int ymax, const int zmin, const int zmax) {
            addNewAMRBox(xmin, xmax, ymin, ymax, zmin, zmax);
            std::string amr_box = convertFromVariadicArgsToString(xmin, xmax, ymin, ymax, zmin, zmax);
            buffer += " amr_box=\"" + amr_box + "\"";
        }

        void setAMRBoxNode(const int xmin, const int xmax, const int ymin, const int ymax, const int zmin, const int zmax) {
            // -1: node to cell index
            addNewAMRBox(xmin, xmax - 1, ymin, ymax - 1, zmin, zmax - 1);
            std::string amr_box = convertFromVariadicArgsToString(xmin, xmax - 1, ymin, ymax - 1, zmin, zmax - 1);
            buffer += " amr_box=\"" + amr_box + "\"";
        }

        void setAMRBoxFromParentIndex(const int index, const int xmin_on_parent, const int xmax_on_parent, const int ymin_on_parent, const int ymax_on_parent, const int zmin_on_parent, const int zmax_on_parent) {
            if (amr_info.current_level == 0) {
                throw std::logic_error("[Simple VTK ERROR] setAMRBoxFromParentIndex() cannot be called on Level 0 Block.");
            }

            if (amr_info.blocks.count(amr_info.current_level - 1) > 0) {
                auto& blocks = amr_info.blocks[amr_info.current_level - 1];

                if (blocks.boxes.count(index) > 0) {
                    const auto& parent_amr_box = blocks.boxes[index];
                    const int xmin = 2 * parent_amr_box.xmin + 2 * xmin_on_parent;
                    const int xmax = 2 * parent_amr_box.xmin + 2 * xmax_on_parent + 1;
                    const int ymin = 2 * parent_amr_box.ymin + 2 * ymin_on_parent;
                    const int ymax = 2 * parent_amr_box.ymin + 2 * ymax_on_parent + 1;
                    const int zmin = 2 * parent_amr_box.zmin + 2 * zmin_on_parent;
                    const int zmax = 2 * parent_amr_box.zmin + 2 * zmax_on_parent + 1;
                    setAMRBox(xmin, xmax, ymin, ymax, zmin, zmax);
                } else {
                    const std::string error_message = "[Simple VTK ERROR] Specified Parent Index " + std::to_string(index) + " does not exist on setAMRBoxFromParentIndex().";
                    throw std::logic_error(error_message);
                }
            } else {
                throw std::logic_error("[Simple VTK ERROR] Parent Block Element did not initialized. Call setAMRBoxFromParentIndex() after Defining parent DataSets.");
            }
        }

        // if parent level specified
        void setAMRBoxFromParentIndex(const int index, const int parent_level, const int xmin_on_parent, const int xmax_on_parent, const int ymin_on_parent, const int ymax_on_parent, const int zmin_on_parent, const int zmax_on_parent) {
            if (amr_info.blocks.count(parent_level) > 0) {
                auto& blocks = amr_info.blocks[parent_level];

                if (blocks.boxes.count(index) > 0) {
                    const auto& parent_amr_box = blocks.boxes[index];
                    const int xmin = 2 * parent_amr_box.xmin + 2 * xmin_on_parent;
                    const int xmax = 2 * parent_amr_box.xmin + 2 * xmax_on_parent + 1;
                    const int ymin = 2 * parent_amr_box.ymin + 2 * ymin_on_parent;
                    const int ymax = 2 * parent_amr_box.ymin + 2 * ymax_on_parent + 1;
                    const int zmin = 2 * parent_amr_box.zmin + 2 * zmin_on_parent;
                    const int zmax = 2 * parent_amr_box.zmin + 2 * zmax_on_parent + 1;
                    setAMRBox(xmin, xmax, ymin, ymax, zmin, zmax);
                } else {
                    const std::string error_message = "[Simple VTK ERROR] Specified Parent Index " + std::to_string(index) + " does not exist on setAMRBoxFromParentIndex().";
                    throw std::logic_error(error_message);
                }
            } else {
                throw std::logic_error("[Simple VTK ERROR] Parent Block Element did not initialized. Call setAMRBoxFromParentIndex() after Defining parent DataSets.");
            }
        }

        void setAMRBoxNodeFromParentIndex(const int index, const int xmin_on_parent, const int xmax_on_parent, const int ymin_on_parent, const int ymax_on_parent, const int zmin_on_parent, const int zmax_on_parent) {
            setAMRBoxFromParentIndex(index, xmin_on_parent, xmax_on_parent - 1, ymin_on_parent, ymax_on_parent - 1, zmin_on_parent, zmax_on_parent - 1);
        }

        void setAMRBoxNodeFromParentIndex(const int index, const int parent_level, const int xmin_on_parent, const int xmax_on_parent, const int ymin_on_parent, const int ymax_on_parent, const int zmin_on_parent, const int zmax_on_parent) {
            setAMRBoxFromParentIndex(index, parent_level, xmin_on_parent, xmax_on_parent - 1, ymin_on_parent, ymax_on_parent - 1, zmin_on_parent, zmax_on_parent - 1);
        }

        //! Inner array inserters
        template<typename T>
        void addArray(const T* values_ptr, const int N) {
            beginInnerElement();

            std::stringstream ss;
            for(size_t i = 1; i <= N; ++i) {
                ss << values_ptr[i - 1];

                if (i < N) {
                    ss << " ";

                    if (i % innerElementPerLine == 0) {
                        buffer += ss.str() + newLine;
                        insertIndent();
                        ss.str("");
                        ss.clear(std::stringstream::goodbit);
                    }
                }
            }

            if (ss.str() != "") {
                buffer += ss.str() + newLine;
            }

            endInnerElement();
        }

        template<typename T>
        void add2DArray(T** values_ptr, const int nx, const int ny) {
            beginInnerElement();

            std::stringstream ss;

            size_t itr = 1;
            size_t size = nx * ny;

            for(size_t j = 1; j <= ny; ++j) {
                for(size_t i = 1; i <= nx; ++i) {
                    ss << values_ptr[i - 1][j - 1];

                    if (itr < size) {
                        ss << " ";

                        if (itr % innerElementPerLine == 0) {
                            buffer += ss.str() + newLine;
                            insertIndent();
                            ss.str("");
                            ss.clear(std::stringstream::goodbit);
                        }
                    }

                    ++itr;
                }
            }

            if (ss.str() != "") {
                buffer += ss.str() + newLine;
            }

            endInnerElement();
        }

        template<typename T>
        void add3DArray(T*** values_ptr, const int nx, const int ny, const int nz) {
            beginInnerElement();

            std::stringstream ss;

            size_t itr = 1;
            size_t size = nx * ny * nz;

            for(size_t k = 1; k <= nz; ++k) {
                for(size_t j = 1; j <= ny; ++j) {
                    for(size_t i = 1; i <= nx; ++i) {
                        ss << values_ptr[i - 1][j - 1][k - 1];

                        if (itr < size) {
                            ss << " ";

                            if (itr % innerElementPerLine == 0) {
                                buffer += ss.str() + newLine;
                                insertIndent();
                                ss.str("");
                                ss.clear(std::stringstream::goodbit);
                            }
                        }

                        ++itr;
                    }
                }
            }

            if (ss.str() != "") {
                buffer += ss.str() + newLine;
            }

            endInnerElement();
        }

        template<typename T, size_t N>
        void addArray(const std::array<T, N>& values) {
            beginInnerElement();

            std::stringstream ss;
            for(size_t i = 1; i <= N; ++i) {
                ss << values[i - 1];

                if (i < N) {
                    ss << " ";

                    if (i % innerElementPerLine == 0) {
                        buffer += ss.str() + newLine;
                        insertIndent();
                        ss.str("");
                        ss.clear(std::stringstream::goodbit);
                    }
                }
            }

            if (ss.str() != "") {
                buffer += ss.str() + newLine;
            }

            endInnerElement();
        }

        template<typename T>
        void addVector(const std::vector<T>& values) {
            beginInnerElement();

            std::stringstream ss;
            auto size = values.size();
            for(size_t i = 1; i <= size; ++i) {
                ss << values[i - 1];

                if (i < size) {
                    ss << " ";

                    if (i % innerElementPerLine == 0) {
                        buffer += ss.str() + newLine;
                        insertIndent();
                        ss.str("");
                        ss.clear(std::stringstream::goodbit);
                    }
                }
            }

            if (ss.str() != "") {
                buffer += ss.str() + newLine;
            }

            endInnerElement();
        }

#ifdef USE_BOOST
        // based on c_index_order (row-major)
        template<typename T, size_t N>
        void addMultiArray(const boost::multi_array<T, N>& values, const bool isFortranStorageOrder = false) {
            const int size = values.num_elements();

            if (isFortranStorageOrder) {
                addArray(values.data(), size);
            } else {
                using array_type = boost::multi_array<T, N>;
                boost::array<typename array_type::index, N> index;

                // initialize index
                for(int i = 0; i < N; ++i) {
                    index[i] = 0;
                }
                auto shape = values.shape();

                // capture index and shape
                std::function<void(int)> proceedIndex = [&index, &shape, &proceedIndex](const int axis){
                    if (axis < N) {
                        // increments from the most left index
                        index[axis] += 1;
                        if (index[axis] == shape[axis]) {
                            index[axis] = 0;
                            proceedIndex(axis + 1);
                        }
                    }
                };

                beginInnerElement();
                std::stringstream ss;
                for (size_t i = 1; i <= size; ++i) {
                    ss << values(index);
                    proceedIndex(0);

                    if (i < size) {
                        ss << " ";

                        if (i % innerElementPerLine == 0) {
                            buffer += ss.str() + newLine;
                            insertIndent();
                            ss.str("");
                            ss.clear(std::stringstream::goodbit);
                        }
                    }
                }

                if (ss.str() != "") {
                    buffer += ss.str() + newLine;
                }

                endInnerElement();
            }
        }
#endif

        template<typename... Args>
        void addItem(Args&&... args) {
            beginInnerElement();

            std::stringstream ss;
            addItemInternal(ss, std::forward<Args>(args)...);
            buffer += ss.str() + newLine;

            endInnerElement();
        }

        // BaseExtent Setting Functions
        void changeBaseExtent(const int xmin, const int xmax, const int ymin, const int ymax, const int zmin, const int zmax) {
            base_extent.xmin = xmin;
            base_extent.xmax = xmax;
            base_extent.ymin = ymin;
            base_extent.ymax = ymax;
            base_extent.zmin = zmin;
            base_extent.zmax = zmax;
            manage_extent_status.extent_initialized = true;
        }

        template<typename T>
        void changeBaseOrigin(const T x0, const T y0, const T z0) {
            base_extent.x0 = x0;
            base_extent.y0 = y0;
            base_extent.z0 = z0;
            manage_extent_status.origin_initialized = true;
        }

        template<typename T>
        void changeBaseSpacing(const T dx, const T dy, const T dz) {
            base_extent.dx = dx;
            base_extent.dy = dy;
            base_extent.dz = dz;
            manage_extent_status.spacing_initialized = true;
        }

        template<typename T>
        void changeRefinementRatio(const T new_ratio) {
            amr_info.refinement_ratio = new_ratio;
        }

        ExtentInfo_t getBaseExtent() const {
            return base_extent;
        }

        void enableExtentManagement() { manage_extent_status.enable = true; }
        void disableExtentManagement() { manage_extent_status.enable = false; }
        bool isExtentManagementEnable() const {
            return (manage_extent_status.enable && manage_extent_status.extent_initialized && manage_extent_status.origin_initialized && manage_extent_status.spacing_initialized);
        }

        // helper functoins
        template<typename T>
        void addDataArray(const std::string name, const std::string number_type, const std::string format, const std::vector<T>& values) {
            beginDataArray(name, number_type, format);
            addVector<T>(values);
            endDataArray();
        }

        template<typename T>
        void addPointScalars(const std::string name, const std::string number_type, const std::string format, const std::vector<T>& values) {
            beginPointData();
            setScalars(name);
            addDataArray(name, number_type, format, values);
            endPointData();
        }

        template<typename T>
        void addCellScalars(const std::string name, const std::string number_type, const std::string format, const std::vector<T>& values) {
            beginCellData();
            setScalars(name);
            addDataArray(name, number_type, format, values);
            endCellData();
        }
};

#endif