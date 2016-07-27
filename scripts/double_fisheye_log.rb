#! /usr/bin/env ruby
# -*- coding: utf-8 -*-
# If you want to start the Microsoft Life Cam or the Gumstix camera e-CAM32
# you should use the corresponding ruby run-script. 

if ARGV.empty?
    puts "ERROR: missing argument: You have to inform the log path"
    exit
end

require 'orocos'
require 'vizkit'
include Orocos
Orocos.initialize

Orocos.run \
        'fisheye::SplitImage' => 'FSI', 
        'projection::DepthMapToImage' => 'DMI',
        'projection::ColorizePointcloudMultiFishEyeCam' => 'CPMF'        do
            
#    Orocos.log_all_ports
    log = Orocos::Log::Replay.open(ARGV[0])
#    camera = TaskContext.get 'camera_usb_deployment'  
    fsi = TaskContext.get 'FSI'
    fsi.configure
    fsi.start
    
    dmi = TaskContext.get 'DMI'
    dmi.configure
    dmi.start
    

    cpmf = TaskContext.get 'CPMF'
    cpmf.pc2Cam1.orientation.re    = -0.7071067812
    cpmf.pc2Cam1.orientation.im[0] =  0.7071067812
    cpmf.pc2Cam2.orientation.im[1] =  0.7071067812
    cpmf.pc2Cam2.orientation.im[2] =  0.7071067812
    cpmf.apply_conf_file('./cpmf.yml',['default'])
    cpmf.configure
    cpmf.start


    log.camera_usb_deployment.frame.connect_to fsi.input_image
    
    fsi.left_image.connect_to cpmf.camera1
    fsi.right_image.connect_to cpmf.camera2
    
    log.velodyne.laser_scans.connect_to cpmf.depthMap
    log.velodyne.laser_scans.connect_to dmi.depth_map
    
    
#    Vizkit.display log.velodyne.laser_scans
#    Vizkit.display cpmf.colored_points
#    imgL = Vizkit.display fsi.left_image
#    imgR = Vizkit.display fsi.right_image
    imgL = Vizkit.display cpmf.camera1_out
    imgR = Vizkit.display cpmf.camera2_out
    imgL.connect(SIGNAL('clickedImage(const QPoint&)')) do |point|
      puts  "L:(#{point.x}w,#{point.y}h)"
    end
    imgR.connect(SIGNAL('clickedImage(const QPoint&)')) do |point|
      puts  "R:(#{point.x}w,#{point.y}h)"
    end

    begin
        Vizkit.control log
        Vizkit.exec
    rescue Interrupt => e
        puts 'interrupted'
    end

    STDERR.puts "shutting down"
end
