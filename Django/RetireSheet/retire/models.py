from django.db import models
# Create your models here.
class User(models.Model):
    name = models.CharField(max_length = 20)
    email = models.EmailField()
    
    def __str__(self): 
        return 'User: %s, E-mail: %s' % (self.name, self.email)
    
class Category(models.Model):
    category = models.CharField(max_length = 20)
    owner = modles.ForeignKey(User)
    
    def __str__(self): 
        return 'Owner: %s, Ticker: %s, Category: %s' % (self.owner, self.category)
    
class Ticker(models.Model):
    ticker = models.CharField(max_length = 20)
    modified = models.DateField()

    def __str__(self): 
        return 'Ticker: %s, Last modified Date: %s' % (self.ticker, self.modified)
        
class DayTrade(models.Model):
    ticker = modles.ForeignKey(Ticker)
    date    = models.DateField()
    open_price = models.FloatField()
    high    = models.FloatField()
    low	    = models.FloatField()
    close   = models.FloatField()	
    volume  = models.IntegerField()
    adj_close = models.FloatField()
    
    def __str__(self): 
        return 'Ticker: %s, Date info: %s' % (self.ticker, self.date)

class EPS(models.Model):
    ticker = modles.ForeignKey(Ticker)
    date   = models.DateField()
    eps    = models.FloatField()
    
    def __str__(self): 
        return 'Ticker: %s, Date : %s, EPS:%s' % (self.ticker, self.date, self.eps)