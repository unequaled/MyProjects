# -*- coding: utf-8 -*-

from django.shortcuts import render

from django.http import JsonResponse #https://docs.djangoproject.com/en/dev/ref/request-response/#jsonresponse-objects
from django.http import HttpResponse
from django.views.decorators.csrf import csrf_exempt
import urllib2
import json


def index(request):
    '''
        For processing, the regular webpage
    '''
    return render(request,'index.html')


def retire(request):
    '''
        For processing, the retire webpage
    ''' 
    return render(request,'retire.html')

def ingredients(request):
    if request.is_ajax():
        if request.method == 'GET':
            data = file_load("static/static/json/data.txt")
            # Javascript only use "" as a string 
            return HttpResponse(str(data).replace("'", '"'))
            
        elif request.method == 'POST':
            print request.body
            return HttpResponse('ok')
    return HttpResponse('No~')
   

def trashmap(request):
    '''
        For processing, the regular webpage
    '''
    return render(request,'trashmap.html')
    
@csrf_exempt
def mapsdata(request, city, dist):
    
    if city == u'\u53f0\u5317\u5e02':
        data = file_load("static/static/json/taipei.json")
    else:
        data = file_load("static/static/json/new_taipei.json")
     
    data_out = json.loads(data)
    output = []

    # print data_out[0]
    
    # print type(data_out[0][u'Region']) == type(dist)
    # print data_out[0][u'Region'] == dist
  
    # for key in data_out[0]:
    #     print key.decode('utf-8')
        
    for item in data_out:
        if item['Region'] == dist:
            output.append(item)

    return HttpResponse(str(json.dumps(output)).replace("'", '"'))
    
    
    # if request.method == 'GET':
    #     data = file_load("static/static/json/test.json")
    #     # Javascript only use "" as a string 
        
    #     return HttpResponse(str(data).replace("'", '"'))
    
    # elif request.method == 'POST':
    #     myDict = dict(request.POST.iterlists())
    #     # print myDict
       
    #     # You can't print big5 string on Koding 
    #     # It will fault
    #     city = myDict.values()[0][0]
    #     region =  myDict.values()[1][0]
        
    #     if city == u'\u53f0\u5317\u5e02':
    #         # Taipei
    #         data = file_load("static/static/json/test.json")
    #     else:
    #         data = file_load("static/static/json/test.json")
            
    #     return HttpResponse(str(data).replace("'", '"'))
        
   


def file_save(data,path):
    '''
        Save the data as json data form.
    '''
    try :
        with open(path, 'w') as outfile:    
            json.dump(data, outfile)
    except:
        print "save error!!!"

def file_load (path):  
    '''
        Load the data as json data form.
    '''
    try :
        # with open(path) as data_file:    
        #     data = yaml.safe_load(data_file)
        f = open(path, 'r')
        data = f.read()
        f.close()
    except:
        print "load error!!!"
        data = None
    return data

