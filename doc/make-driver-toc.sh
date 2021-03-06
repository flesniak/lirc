#!/usr/bin/env bash

cat << EOT
<?xml version="1.0"?>
<xsl:stylesheet version="1.0"
    xmlns:xsl="http://www.w3.org/1999/XSL/Transform">

<xsl:template match="ul[@class='driver-toc']">
  <UL TYPE="disc">
EOT

for f in $@; do
    basename=$(basename $f .html)
    echo "     <LI><A HREF=\"$basename.html\">${basename}</A></LI>"
done

cat << EOT
  </UL>
  <UL class="ext-driver-toc"/>
</xsl:template>
</xsl:stylesheet>
EOT
