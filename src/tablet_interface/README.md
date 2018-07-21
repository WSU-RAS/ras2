# tablet-interface
Tablet interface using rosbridge and a web server

## Installing

    sudo apt install ros-kinetic-rosbridge-server ros-kinetic-rosbridge-suite \
        python-tornado python-bson nginx
    sudo cp ras-nginx-config.txt /etc/nginx/sites-available/ras
    sudo unlink /etc/nginx/sites-enabled/default
    sudo ln -s /etc/nginx/sites-{available,enabled}/ras
    sudo systemctl restart nginx

If you get permission denied, then set your home directory to be readable:

    chmod 755 ~/
