from django.db import models
# Create your models here.
'''
class Route(models.Model):
    name = models.CharField(max_length = 20)
    def __str__(self): 
        return 'Route name: %s' % (self.name)

class City(models.Model):
    name = models.CharField(max_length = 20)
    def __str__(self): 
        return 'City name: %s' % (self.name)
        
class District(models.Model):
    name = models.CharField(max_length = 20)
    def __str__(self): 
        return 'District name: %s' % (self.name)
        
class Location(models.Model):
    name = models.CharField(max_length = 20)
    city = models.ForeignKey(City)
    district = models.ForeignKey(District)
    route = models.ForeignKey(Route)
    lng = models.FloatField()
    lat = models.FloatField()
    time = models.TimeField()
    garbage = models.CharField(max_length = 20)
    recycle = models.CharField(max_length = 20)
    good = models.CharField(max_length = 20)
'''