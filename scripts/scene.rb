require 'zip/zip'
require 'set'
require 'eigen'
require 'rexml/document'
require 'envire/gdal'
require 'utilrb/kernel'

class MarsScene
    attr_reader :path
    attr_reader :entries
    attr_reader :zipfile
    attr_reader :scene

    attr_reader :root_node
    attr_reader :node_list
    attr_reader :material_list

    def unique_element(name, root = scene)
        candidates = REXML::XPath.each(root, name).to_a
        if candidates.size > 1
            raise ArgumentError, "multiple #{name} elements are not supported"
        elsif candidates.empty?
            root.add_element(name)
        else
            candidates.first
        end
    end

    def initialize(path, write_mode = false)
        @path    = path
        if !write_mode
            @zipfile = Zip::ZipFile.open(path)
            @entries = Zip::ZipFile.enum_for(:foreach, path).to_set

            scene_entry = entries.find { |p| File.extname(p.to_s) == ".scene" }
            @scene         = REXML::Document.new(zipfile.get_input_stream(scene_entry.to_s) { |s| s.read })
            @root_node     = unique_element('SceneFile')
            @node_list     = unique_element('nodelist', root_node)
            @material_list = unique_element('materiallist', root_node)
                
            @node_list_index = REXML::XPath.each(node_list, 'node/index').inject(0) do |value, node|
                [value, Integer(node.text)].max
            end + 1
            @material_list_index = REXML::XPath.each(material_list, 'material/id').inject(0) do |value, node|
                [value, Integer(node.text)].max
            end + 1
        else
            @zipfile = Zip::ZipFile.open(path, Zip::ZipFile::CREATE)
            @scene = REXML::Document.new
            @root_node           = @scene.add_element('SceneFile')
            @node_list           = @root_node.add_element('nodelist')
            @material_list       = @root_node.add_element('materiallist')
            @node_list_index     = 1
            @material_list_index = 1
        end
    end

    def write
        scene_filename = "#{File.basename(path, File.extname(path))}.scene"
        scene_io = StringIO.new
        @scene.write(scene_io, 4)
        scene_data = scene_io.string
        add_file(scene_filename, scene_data)
        @zipfile.commit
    end

    def each_terrain
        REXML::XPath.each(@node_list, 'node/t_srcname').each do |node|
            node = node.parent

            filename = node.get_elements('t_srcname').first.text
            pos      = node.get_elements('position').first
            coordinates = %w{xpos ypos zpos}.map { |el| Float(pos.get_elements(el).first.text) }
            pos = Eigen::Vector3.new(*coordinates)

            coordinates = %w{t_width t_height t_scale}.map { |el| Float(node.get_elements(el).first.text) }
            size = Eigen::Vector3.new(*coordinates)

            yield(filename, pos, size)
        end
    end

    def get_file(filename)
        zipfile.get_input_stream(filename)
    end

    def add_file(filename, data = nil, &block)
        zipfile.get_output_stream(filename) do |io|
            yield(io) if block_given?
            io.write data
        end
    end

    def add_node(name)
        node = @node_list.add_element('node', 'name' => name)
        node.add_element('index').add_text(@node_list_index.to_s)
        @node_list_index += 1
        node
    end

    def add_array(data, root_node, *names)
        data.each_with_index do |v, i|
            root_node.add_element(names[i]).add_text(v.to_s)
        end
    end

    def add_terrain(name, data_file, position, scale, size, texture_file = nil)
        material_id = add_material
        new_node = add_node(name)
        new_node.add_element('origname').add_text("PRIMITIVE")
        new_node.add_element('filename').add_text(data_file.to_s)
        new_node.add_element('t_srcname').add_text(data_file.to_s)
        new_node.add_element('physicmode').add_text("7")
        new_node.add_element('movable').add_text("false")
        new_node.add_element('material_id').add_text(material_id.to_s)
        new_node.add_element('shadow_id').add_text("1")

        position = [position.x + size.x / 2, position.y + size.y / 2, position.z]
        add_array(position,
            new_node.add_element("position"),
            "xpos", "ypos", "zpos")
        add_array(size.to_a,
            new_node.add_element("extend"),
            "extx", "exty", "extz")
        add_array(size.to_a, new_node,
            "t_width", "t_height", "t_scale")
        new_node
    end

    MARS_BAND_COLORS = [Gdal::Gdalconst::GCI_REDBAND,
        Gdal::Gdalconst::GCI_GREENBAND,
        Gdal::Gdalconst::GCI_BLUEBAND]

    # Takes a georeferenced GDAL terrain file as input and formats it in a way
    # that can be used by Mars
    def self.convert_terrain_from_gdal(input_path, output_path)
        source_map = Gdal::Gdal.open(input_path)
        source_data = source_map.read_band(1)
        min, max = [source_data.min, source_data.max]
        # Add 1, and use 65000 as the max scale range to make sure that we don't
        # overflow the possible values
        min -= 1
        scale = 65000.0 / (max - min)
        transform = source_map.get_geo_transform
        if transform[2] != 0 || transform[4] != 0
            raise ArgumentError, "cannot handle rotated grids"
        end

        x0, y0, x_scale, y_scale = transform[0], transform[3], transform[1], transform[5]
        mapped_data = source_data.map { |v| Integer((v - min) * scale) }.
            each_slice(source_map.xsize).map(&:reverse).inject([], &:concat)

        # GDAL-Ruby does not offer a way to delete a dataset (that I know of),
        # but that's the only way to make sure that the data is completely
        # flushed on disk
        #
        # Fork to generate the dataset
        pid = fork do
            driver = Gdal::Gdal.get_driver_by_name('GTiff')
            target_map = driver.create(output_path, source_map.xsize, source_map.xsize, 3, Gdal::Gdalconst::GDT_UINT16, ["PHOTOMETRIC=RGB"])
            3.times do |i|
                band = target_map.band(i + 1)
                target_map.write_band(i + 1, mapped_data)
            end
        end
        Process.wait(pid)

        x_size = source_map.xsize * x_scale
        y_size = source_map.xsize * y_scale
        z_size = max - min

        return Eigen::Vector3.new(x0, y0, min),
            Eigen::Vector3.new(x_scale, y_scale, 1.0 / (max - min)),
            Eigen::Vector3.new(x_size, y_size, z_size)
    end

    COLOR_SUFFIXES = %w{r g b a}
    def add_color(element, r, g, b, a)
        basename = element.name
        [r, g, b, a].each_with_index do |val, i|
            element.add_element("#{basename}#{COLOR_SUFFIXES[i]}").add_text(val.to_s)
        end
    end

    def add_material(options = Hash.new)
        options = Kernel.validate_options options,
            :diffuse => [1.0, 1.0, 1.0, 1.0],
            :specular => [0.1, 0.1, 0.1, 1.0],
            :shininess => 2.0,
            :texture => nil,
            :bumpmap => nil,
            :tex_scale => nil

        id = @material_list_index
        @material_list_index += 1
        new_material = @material_list.add_element "material"
        new_material.add_element("id").add_text(id.to_s)
        add_color(new_material.add_element("diffuseFront"), *options[:diffuse])
        add_color(new_material.add_element("specularFront"), *options[:specular])
        new_material.add_element("shininess").add_text(options[:shininess].to_s)
        if options[:texture]
            new_material.add_element("texturename", options[:texture])
        end
        if options[:bumpmap]
            new_material.add_element("bumpmap", options[:bumpmap])
        end
        id
    end
end

