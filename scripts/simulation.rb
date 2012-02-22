require 'orocos'
require 'readline'
include Orocos

Orocos.initialize

Orocos.run 'simulation', "valgrind" => false, :wait => 10 do

    mars = TaskContext.get 'mars_simulation'
    mars.controller_port = 1600
    mars.enable_gui = 1

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

    Readline::readline("Press ENTER to quit") do
    end

end 
