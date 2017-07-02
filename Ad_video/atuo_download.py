# -*- coding: utf-8 -*-
import urllib
test=urllib.FancyURLopener()
video_url = 'http://video.tudou.com/v/XMTg5MzI1NjM4MA==.html'
test.retrieve("http://mp.weixin.qq.com/s?__biz=MjM5MDQxMTgzMw==&mid=2651813743&idx=1&sn=98be1a73772b45ea6c8ba3ffb10e53a6&chksm=bdbeea398ac9632fce0709e256b0a1eda912108005c7533314517368f42622ae7cbd40964f92&mpshare=1&scene=5&srcid=0223HWwN2HYvlGgnRUz4d6Yk#rd","testout.mp4")