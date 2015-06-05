from django.shortcuts import render

from django.http import JsonResponse #https://docs.djangoproject.com/en/dev/ref/request-response/#jsonresponse-objects
from django.http import HttpResponse
import json
import yaml

def plot(request):
    '''
        For processing, the retire webpage
    ''' 
    return render(request,'plot.html')


def base(request):
    '''
        For processing, the retire webpage
    ''' 
    return render(request,'base_boostrap.html')

def retire(request):
    '''
        For processing, the retire webpage
    ''' 
    return render(request,'retire.html')

def index(request):
    '''
        For processing, the regular webpage
    '''
    return render(request,'index.html')


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
        with open(path) as data_file:    
            data = yaml.safe_load(data_file)
    except:
        print "load error!!!"
        data = None
    return data

