from django.shortcuts import render
from django.http import HttpResponse
from datetime import datetime,timedelta
import urllib2

#Need to setup the build packget first 
#apt-get install build-essential python-dev    
#pip install numpy, pandas,html5,wheel,lxml,BeautifulSoup4 
import html5lib
import numpy as np
import pandas as pd
from pandas import Series, DataFrame
from pandas import read_html
from pandas.io.data import DataReader

from mydatabase.models import Ticker,DayTrade,EPS

# Create your views here.
def index(request, string):
    if string == 'ABT':
        string += update(string)
        
    return HttpResponse('Here i am ' + string)
    
    
def update(symbol):
    try:
        obj_ticker = Ticker.objects.get(ticker=symbol)
        update = True
        
    except:
        obj_ticker = Ticker(ticker=symbol, modified = datetime.now())
        obj_ticker.save()
        update = False
    
    if update:
        eps_table_update(obj_ticker,obj_ticker.modified)
        ticker_table_update(obj_ticker,obj_ticker.modified)
        return "<br> found"
    else:
        eps_table_update(obj_ticker)
        ticker_table_update(obj_ticker)
        return "<br> not found"

def eps_table_update(key_ticker,date = None):
    
    if(EPS.objects.filter(ticker=key_ticker).order_by('-date')[0].date + timedelta(90) > datetime.now().date())
        # Today - lastest EPS date < 90 breal;
        return 
    
    # Parser the date from web to pandas form
    data = pd.io.html.read_html(url_eps_data(key_ticker.ticker))
    eps  = pd.concat([data[0].dropna(),data[1].dropna()]).reset_index(drop='True')
    eps  = eps.replace({'March':'Mar','June':'Jun','Sept.':'Sep','Dec.':'Dec'},regex=True)
    eps[0] = pd.to_datetime(eps[0], format='%b %d, %Y')     #This code transfer string to date object            
    eps.columns = ['Date','EPS']
    
    for index in range(eps.shape[0]):
        if date is None or date < eps.iloc[index][0].date():
            try:
                # Check the entry is already existed in database.
                new_eps = EPS.objects.get(ticker = key_ticker ,date = eps.iloc[index][0].date())
            except:
                new_eps = EPS(ticker = key_ticker ,date = eps.iloc[index][0],eps = round(eps.iloc[index][1], 2))
                new_eps.save();
                
        else:
            break
    
def ticker_table_update(key_ticker,date = None):
    if date is None:
        date = EPS.objects.filter(ticker=key_ticker).order_by('date')[0].date
    
    try:
        # If there is no date can retrieve from website
        # Which means no date updat required.
        table = DataReader(key_ticker.ticker ,'yahoo',date,datetime.now()-timedelta(1))
        table.reset_index(level=0, inplace=True)
        for index in range(table.shape[0]):
            try:
                # Check the entry is already existed in database.
                new_daytrade = DayTrade.objects.get(ticker=key_ticker, date = table.iloc[index][0].date())
            except:
                new_daytrade = DayTrade(ticker      = key_ticker,
                                            date        = table.iloc[index][0],
                                            open_price  = table.iloc[index][1],
                                            high        = table.iloc[index][2],
                                            low	        = table.iloc[index][3],
                                            close       = table.iloc[index][4],
                                            volume      = table.iloc[index][5],
                                            adj_close   = table.iloc[index][6] )
                new_daytrade.save() 
        return "<br>Update completed"
    except:
        return "<br>No update required"

def url_eps_data(symbol):
    '''
        To get the eps from website
    '''
    return 'https://ycharts.com/companies/' + symbol + '/eps'