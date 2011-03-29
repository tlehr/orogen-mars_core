require 'orocos'
include Orocos

Orocos.initialize

Orocos::Process.spawn 'simulation', "valgrind" => false, "wait" => 1000 do |process|

    mars = process.task 'mars_simulation'

    mars.resource_dir="#{ENV['ROCK_INSTALL_PATH']}/share/mars/resources/"

    mars.stuff_path = "/opt/workspace/rimres/migration/install/share/mars/resources/"
    mars.gui_path = "/opt/workspace/rimres/migration/install/share/mars/resources/"
    mars.tmp_path = "/opt/workspace/rimres/migration/install/share/mars/resources/tmp/"
    mars.save_path ="/opt/workspace/rimres/migration/install/share/mars/resources/save/"
    mars.plugin_path= "/opt/workspace/rimres/migration/install/share/mars/"
    mars.debug_path ="/opt/workspace/rimres/migration/install/share/mars/resources/"

    mars.controller_port = 1600

#    option_t = Orocos.registry.get 'simulation/Option'
#    option = option_t.new
#    option.name = "-c"
#    option.parameter = "1601"
#
#    raw_options = mars.raw_options
#    raw_options << option
#
#    mars.raw_options = raw_options

    mars.configure
    mars.start

    while true
	sleep 0.1
    end
end 
