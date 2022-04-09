#!/usr/bin/env python3

import http.server, ssl
import functools
#import webbrowser

server_address = ('', 8080)
ssl_key_file = "/etc/ssl/certs/localcerts/server1.example.com.key"
ssl_certificate_file = "/etc/ssl/certs/localcerts/server1.example.com.pem"

Handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory='../.././')

httpd = http.server.HTTPServer(server_address, Handler)
httpd.socket = ssl.wrap_socket(httpd.socket,
                               server_side=True,
                               keyfile=ssl_key_file,
                               certfile=ssl_certificate_file,
                               ssl_version=ssl.PROTOCOL_TLS)
httpd.serve_forever()
    
#webbrowser.open('https://127.0.0.1:8080', new=2)
