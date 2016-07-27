#! /usr/bin/env ruby
# -*- coding: utf-8 -*-
# If you want to start the Microsoft Life Cam or the Gumstix camera e-CAM32
# you should use the corresponding ruby run-script. 

require 'orocos'
require 'vizkit'
include Orocos
Orocos.initialize

Orocos.run \
        'camera_usb_deployment', 
        "velodyne_lidar::LaserScanner" => "velodyne",
        "velodyne_lidar::Positioning" => "positioning" \
        do
#        'fisheye::SplitImage' => 'FSI', 
#        'projection::ColorizePointcloudMultiFishEyeCam' => 'CPMF'        do
    Orocos.log_all_ports
    camera = TaskContext.get 'camera_usb_deployment'  
#    fsi = TaskContext.get 'FSI'
#    fsi.configure
#    fsi.start

#    cpmf = TaskContext.get 'CPMF'
#    cpmf.configure
#    cpmf.start

    camera.camera_device = "/dev/video0"
    
    camera.camera_format = :MODE_RGB
    camera.output_format = :MODE_RGB
    camera.width = 2560
    camera.height = 720

    velodyne = TaskContext.get 'velodyne'
    velodyne.configure
    velodyne.start
    
    pos = TaskContext.get 'positioning'
    pos.configure
    pos.start


    camera.configure
    camera.start


#    camera.frame.connect_to fsi.input_image
    
#    fsi.left_image.connect_to cpmf.camera1
#    fsi.right_image.connect_to cpmf.camera2
    
#    velodyne.laser_scans.connect_to cpmf.depthMap
    
    Vizkit.display camera.frame
    Vizkit.display velodyne.laser_scans
#    imgL = Vizkit.display fsi.left_image
#    imgR = Vizkit.display fsi.right_image
    #imgL.connect(SIGNAL('clickedImage(const QPoint&)')) do |point|
    # puts  "L:(#{point.x}w,#{point.y}h)"
    #end
    #imgR.connect(SIGNAL('clickedImage(const QPoint&)')) do |point|
    # puts  "R:(#{point.x}w,#{point.y}h)"
    #end

    begin
        Vizkit.exec
    rescue Interrupt => e
        velodyne.stop
        positioning.stop
    end

    STDERR.puts "shutting down"
end
